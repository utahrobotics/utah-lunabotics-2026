use wgsl_pcl::shader_module_descriptors_from_wgsl;

#[test]
fn test_shader_module_descriptors_from_wgsl() {
    let descriptors = shader_module_descriptors_from_wgsl!("../shaders/test_shader1.wgsl");

    assert_eq!(descriptors.len(), 1);
}
