#![feature(iter_intersperse)]

use std::{
    panic::{catch_unwind, AssertUnwindSafe},
    str::FromStr,
};

use fxhash::FxHashMap;
use gputter_core::{get_device, init_gputter_blocking, wgpu, GpuDevice};
use proc_macro::TokenStream;
use quote::{format_ident, quote};
use regex::Regex;
use syn::{
    parse::{Parse, ParseStream},
    parse_macro_input,
    token::{self},
    Ident, LitStr, Visibility,
};
use unfmt::unformat;

enum StorageType {
    Uniform,
    Storage { shader_read_only: bool },
}

struct BuildShader {
    vis: Visibility,
    name: Ident,
    _comma: token::Comma,
    shader: LitStr,
}

impl Parse for BuildShader {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        Ok(BuildShader {
            vis: input.parse()?,
            name: input.parse()?,
            _comma: input.parse()?,
            shader: input.parse()?,
        })
    }
}

fn type_resolver(s: &str, uint_consts: &FxHashMap<&str, Option<u32>>) -> String {
    match s {
        "f32" => return "f32".into(),
        "u32" => return "u32".into(),
        "i32" => return "i32".into(),
        "NonZeroU32" => return "std::num::NonZeroU32".into(),
        "NonZeroI32" => return "std::num::NonZeroI32".into(),
        _ => {}
    }

    let delimited = &format!("{s}?");
    if let Some(inner) = unformat!("array<{}>?", delimited) {
        if let Some((ty, mut count)) = unformat!("{},{}", inner) {
            count = count.trim();
            if let Ok(count) = usize::from_str(count) {
                // count is a literal integer
                return format!("[{}; {}]", type_resolver(ty.trim(), uint_consts), count);
            } else if let Some(&count) = uint_consts.get(count) {
                if let Some(count) = count {
                    // count is a constant value
                    return format!("[{}; {}]", type_resolver(ty.trim(), uint_consts), count);
                } else {
                    // If count is None, it is a build time constant that is not known at compile time
                    // of the host code
                    return format!("[{}]", type_resolver(ty.trim(), uint_consts));
                }
            } else {
                panic!("Invalid array count: {count} in {s} {uint_consts:?}");
            }
        }
        // There is no count
        return format!("[{}]", type_resolver(inner.trim(), uint_consts));
    }
    if let Some(inner) = unformat!("atomic<{}>?", delimited) {
        // Atomic types in the shader do not need to be atomic in the host
        return type_resolver(inner.trim(), uint_consts);
    }
    if let Some(mut inner) = unformat!("vec{}?", delimited) {
        inner = inner.trim();
        if let Some((n, ty)) = unformat!("{}<{}>", inner) {
            return format!("gputter::types::AlignedVec{n}<{ty}>");
        } else if inner.len() != 2 {
            panic!("Invalid vector type: {s}");
        } else {
            let (n, mut ty) = inner.split_at(1);
            ty = match ty {
                "f" => "f32",
                "u" => "u32",
                "i" => "i32",
                _ => panic!("Invalid vector type: {s}"),
            };
            return format!("gputter::types::AlignedVec{n}<{ty}>");
        }
    }
    if let Some((dim1, dim2)) = unformat!("mat{}x{}?", delimited) {
        if let Some((dim2, ty)) = unformat!("{}<{}>", dim2.trim_end()) {
            return format!("gputter::types::AlignedMatrix{dim1}x{dim2}<{ty}>");
        } else if dim2.len() != 2 {
            panic!("Invalid matrix type: {s}");
        } else {
            let (dim2, mut ty) = dim2.split_at(1);
            ty = match ty {
                "f" => "f32",
                "u" => "u32",
                "i" => "i32",
                _ => panic!("Invalid matrix type: {s}"),
            };
            return format!("gputter::types::AlignedMatrix{dim1}x{dim2}<{ty}>");
        }
    }

    // It is some custom type that is used verbatim in the shader and the host
    s.into()
}

// Eventually check errors like these
// Shader entry point's workgroup size [36, 24, 1] (864 total invocations) must be less or equal to the per-dimension limit [256, 256, 64] and the total invocation limit 256

#[proc_macro]
pub fn build_shader(input: TokenStream) -> TokenStream {
    // Parse the input tokens into a syntax tree
    let BuildShader {
        vis, name, shader, ..
    } = parse_macro_input!(input as BuildShader);

    // remove comments
    let re = Regex::new(r"//[[[:blank:]]\S]*\n").unwrap();

    let shader = shader.value();
    let shader = re.replace_all(&shader, "\n");

    // Add aliases to the start
    // This helps with later steps if the first symbol in the shader
    // is a buffer annotation
    let shader = {
        let mut tmp = String::with_capacity(shader.len() + 1);
        tmp.push_str("alias NonZeroU32 = u32;\nalias NonZeroI32 = i32;\n");
        tmp.push_str(&shader);
        tmp
    };

    // Find all compute functions
    let re = Regex::new(r"@compute[\s@a-zA-Z0-9\(\)_,\*\+\-/%]+fn\s+([a-zA-Z0-9_]+)\s*\(").unwrap();
    let compute_fns: Vec<_> = re
        .captures_iter(&shader)
        .map(|caps| {
            let (_, [fn_name]) = caps.extract();
            fn_name
        })
        .collect();

    // Find all u32 constants as they can be used for array lengths
    let re =
        Regex::new(r"const\s*([a-zA-Z0-9_]+)\s*:\s*([a-zA-Z0-9]+)\s*=\s*([\{\}a-zA-Z0-9_]+)\s*;?")
            .unwrap();

    let uint_consts: FxHashMap<_, _> = re
        .captures_iter(&shader)
        .filter_map(|caps| {
            let (_, [const_name, const_ty, const_val]) = caps.extract();
            if const_ty != "u32" && const_ty != "NonZeroU32" {
                return None;
            }

            if const_val.starts_with("{{") && const_val.ends_with("}}") {
                return Some((const_name, None));
            }

            let Ok(n) = u32::from_str(&const_val) else {
                panic!(r#"Constant "{const_name}" is not a valid u32 (was {const_val})"#);
            };
            Some((const_name, Some(n)))
        })
        .collect();

    // Split by buffer annotations
    let re = Regex::new(r"#\[buffer\]").unwrap();

    // Parse all buffer definitions
    // They should come immediately after the buffer annotation,
    // and the first split element can be ignore as it is definitely the aliases
    // defined earlier
    let mut buffer_storage_types = vec![];
    let mut buffer_types = vec![];
    let splitted: Vec<_> = re.split(&shader).collect();

    let buffer_names: Vec<_> = splitted
        .iter()
        .skip(1)
        .map(|&s| {
            let (_, storage_ty, name, ty) = unformat!("{}var<{}>{}:{};", s)
                .ok_or_else(|| {
                    panic!(
                        "Format of buffer is incorrect: {}",
                        s.split('\n').next().unwrap_or_default()
                    )
                })
                .unwrap();

            buffer_types.push(type_resolver(ty.trim(), &uint_consts));
            buffer_storage_types.push(if storage_ty.trim() == "uniform" {
                StorageType::Uniform
            } else if let Some((_, shader_rw_mode)) = unformat!("{}storage,{}", storage_ty) {
                match shader_rw_mode.trim() {
                    "read_write" => StorageType::Storage {
                        shader_read_only: false,
                    },
                    "read" => StorageType::Storage {
                        shader_read_only: true,
                    },
                    _ => panic!("Unsupported shader read-write mode: {shader_rw_mode}"),
                }
            } else {
                panic!("Unsupported buffer storage type: {storage_ty}")
            });
            name.trim()
        })
        .collect();

    // Find all build time constants
    // These constants can be filled by the host before the shader is compiled
    let re = Regex::new(
        r"(const\s+[a-zA-Z0-9_]+\s*:\s*([a-zA-Z0-9]+)\s*=\s*)\{\{([a-zA-Z0-9_]+)\}\}\s*(;?)(\s*/!/\s*sub\s*with\s*(\S+))?(\n)?",
    )
    .unwrap();

    let mut const_types = vec![];
    let mut const_names = vec![];
    let mut const_custom_sub = vec![];
    let mut binding_index = 0usize;

    // Prepare shader for substitution
    let shader: Vec<_> = splitted
        .into_iter()
        .map(String::from)
        .intersperse_with(|| {
            let out = format!("<<GRP_SUBSTITUTE{binding_index}?>>");
            binding_index += 1;
            out
        })
        .flat_map(|s| {
            re.captures_iter(&s).for_each(|caps| {
                let mut caps = caps.iter();
                let _whole = caps.next();
                let _declaration = caps.next();
                let type_name = caps.next().unwrap().unwrap().as_str();
                let const_name = caps.next().unwrap().unwrap().as_str();
                let _semicolon = caps.next();
                const_types.push(type_resolver(type_name, &uint_consts));
                const_names.push(const_name.to_owned());
                // panic!("A {:?}", caps.next());
                const_custom_sub.push(
                    caps.next()
                        .flatten()
                        .map(|_| {
                            // If the outer capture group is present, the inner capture group is also present
                            // refer to regex for proof
                            let cap = caps.next().unwrap().unwrap().as_str().trim();
                            if cap.is_empty() {
                                None
                            } else {
                                Some(cap.to_owned())
                            }
                        })
                        .flatten(),
                )
            });
            let mut const_index = 0usize;
            let splitted: Vec<_> = re
                .replace_all(&s, "$1<<SUBSTITUTE>>$4$7")
                .split("<<SUBSTITUTE>>")
                .map(String::from)
                .intersperse_with(|| {
                    let out = format!("<<SUBSTITUTE{}?>>", const_types[const_index]);
                    const_index += 1;
                    out
                })
                .collect();
            splitted
        })
        .collect();

    let mut const_sub_idx = 0usize;

    // Substitute parameters with reasonable defaults
    let tmp_shader: String = shader
        .iter()
        .map(|s| {
            if let Some(i) = unformat!("<<GRP_SUBSTITUTE{}?>>", s) {
                format!("@group(0) @binding({i})")
            } else if let Some(ty) = unformat!("<<SUBSTITUTE{}?>>", s) {
                if let Some(custom_sub) = &const_custom_sub[const_sub_idx] {
                    const_sub_idx += 1;
                    return custom_sub.clone();
                }
                const_sub_idx += 1;
                match ty {
                    "f32" => "0.0".to_owned(),
                    "u32" => "0".to_owned(),
                    "i32" => "0".to_owned(),
                    "bool" => "false".to_owned(),
                    "std::num::NonZeroU32" => "1".to_owned(),
                    "std::num::NonZeroI32" => "1".to_owned(),
                    _ => {
                        if let Some((n, ty)) = unformat!("gputter::types::AlignedVec{}<{}>", ty) {
                            let sub = match ty {
                                "f32" => "0.0".to_owned(),
                                "u32" | "i32" => "0".to_owned(),
                                _ => panic!("Unsupported type for substitution: {}", ty),
                            };
                            return match n {
                                "2" => format!("vec2<{ty}>({sub}, {sub})"),
                                "3" => format!("vec3<{ty}>({sub}, {sub}, {sub})"),
                                "4" => format!("vec4<{ty}>({sub}, {sub}, {sub}, {sub})"),
                                _ => panic!("Unsupported type for substitution: {}", ty),
                            };
                        }
                        panic!("Unsupported type for substitution: {}", ty)
                    }
                }
            } else {
                s.clone()
            }
        })
        .collect();

    // Check that it compiles
    init_gputter_blocking().expect("Failed to initialize gputter");
    let GpuDevice { device, .. } = get_device();
    if let Err(panic) = catch_unwind(AssertUnwindSafe(|| {
        let _ = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: None,
            source: wgpu::ShaderSource::Wgsl((&tmp_shader).into()),
        });
    })) {
        let payload: Box<String> = panic.downcast().unwrap();
        panic!(
            "Failed to compile the following mock shader:\n\n{}\n\n{}",
            tmp_shader, payload
        );
    }

    // Replace substitutions with variable names
    // let mut binding_index = 0usize;
    let mut const_index = 0usize;

    let shader: String = shader
        .iter()
        .map(|s| {
            if let Some(_) = unformat!("<<GRP_SUBSTITUTE{}>>", s) {
                // let out = format!("@group({{{}}}) @binding({{{}.binding_index()}})", binding_index, buffer_names[binding_index]);
                // binding_index += 1;
                // out
                "@group({}) @binding({})".into()
            } else if let Some(_) = unformat!("<<SUBSTITUTE{}>>", s) {
                let out = format!("{{{}}}", const_names[const_index]);
                const_index += 1;
                out
            } else {
                // Replace regular braces with double braces
                s.replace('{', "{{").replace('}', "}}")
            }
        })
        .collect();

    // Define actual buffer types as they appear to the host
    let buffer_def: Vec<_> = buffer_names
        .iter()
        .zip(buffer_types.iter())
        .zip(buffer_storage_types.iter())
        .map(|((&name, ty), storage_ty)| {
            let stream = match storage_ty {
                StorageType::Uniform => format!("pub {name}: gputter::shader::BufferGroupBinding<gputter::buffers::uniform::UniformBuffer<{ty}>, S>"),
                StorageType::Storage { shader_read_only } => {
                    let shader_read_only = if *shader_read_only { "ShaderReadOnly" } else { "ShaderReadWrite" };
                    format!("pub {name}: gputter::shader::BufferGroupBinding<gputter::buffers::storage::StorageBuffer<{ty}, gputter::buffers::storage::HostHidden, gputter::buffers::storage::{shader_read_only}>, S>")
                }
            };
            proc_macro2::TokenStream::from_str(&stream).unwrap()
        })
        .collect();

    // Define actual constant types as they appear to the host
    let const_def = const_names
        .iter()
        .zip(const_types.iter())
        .map(|(name, ty)| {
            proc_macro2::TokenStream::from_str(&format!("pub {name}: {ty}")).unwrap()
        });

    let const_idents = const_names.iter().map(|name| format_ident!("{name}"));
    let buffer_idents = buffer_names.iter().map(|&name| format_ident!("{name}"));
    // let buffer_idents2 = buffer_idents.clone();

    // Create a ComputeFn for each compute function
    let compute_count = compute_fns.len();
    let compile_out = compute_fns.iter().map(|&name| {
        proc_macro2::TokenStream::from_str(&format!(
            "gputter::shader::ComputeFn::new_unchecked(shader.clone(), {name:?}, bind_group_indices.into_boxed_slice())"
        ))
        .unwrap()
    });

    let shader_buffer_sub = buffer_names.iter().map(|&name| {
        proc_macro2::TokenStream::from_str(&format!(
            "bind_group_indices.binary_search(&self.{name}.group_index()).unwrap(), self.{name}.binding_index()"
        ))
        .unwrap()
    });

    // Build the output
    let expanded = quote! {
        #vis struct #name<S> {
            #(#buffer_def,)*
            #(#const_def,)*
        }
        impl<S> #name<S> {
            #vis fn compile(&self) -> [gputter::shader::ComputeFn<S>; #compute_count] {
                // #(let #buffer_idents = &self.#buffer_idents;)*
                let mut bind_group_indices = vec![#(self.#buffer_idents.group_index(), )*];
                bind_group_indices.sort();
                bind_group_indices.dedup();

                #(let #const_idents = &self.#const_idents;)*
                let shader = format!(
                    #shader,
                    #(#shader_buffer_sub,)*
                );

                let shader = gputter::get_device().device.create_shader_module(gputter::wgpu::ShaderModuleDescriptor {
                    label: Some(stringify!(#name)),
                    source: gputter::wgpu::ShaderSource::Wgsl(shader.into()),
                });
                let shader = std::sync::Arc::new(shader);
                [
                    #(#compile_out,)*
                ]
            }
        }
    };

    // Hand the output tokens back to the compiler
    TokenStream::from(expanded)
}
