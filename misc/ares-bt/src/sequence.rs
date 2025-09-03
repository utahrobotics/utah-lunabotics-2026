use crate::{
    Behavior, CancelSafe, EternalBehavior, EternalStatus, FallibleBehavior, FallibleStatus,
    InfallibleBehavior, InfallibleStatus, IntoRon, Status,
};

pub struct Sequence<A> {
    pub body: A,
    index: usize,
}

macro_rules! impl_seq {
    ($len: literal $($name: ident $num: tt)+) => {
        impl<C1, $($name,)+> Behavior<C1> for Sequence<($($name,)+)>
        where
            $($name: Behavior<C1>,)+
        {
            fn run(&mut self, blackboard: &mut C1) -> Status {
                loop {
                    match self.index {
                        $(
                            $num => match self.body.$num.run(blackboard) {
                                Status::Running => return Status::Running,
                                Status::Success => {
                                    self.index += 1;
                                }
                                Status::Failure => {
                                    self.index = 0;
                                    return Status::Failure;
                                }
                            }
                        )+
                        _ => {
                            self.index = 0;
                            return Status::Success;
                        }
                    }
                }
            }
        }
        impl<$($name,)+> CancelSafe for Sequence<($($name,)+)>
        where
            $($name: CancelSafe,)+
        {
            fn reset(&mut self) {
                self.index = 0;
                $(
                    self.body.$num.reset();
                )+
            }
        }
        impl<C1, $($name,)+> InfallibleBehavior<C1> for Sequence<($($name,)+)>
        where
            $($name: InfallibleBehavior<C1>,)+
        {
            fn run_infallible(&mut self, blackboard: &mut C1) -> InfallibleStatus {
                loop {
                    match self.index {
                        $(
                            $num => match self.body.$num.run_infallible(blackboard) {
                                InfallibleStatus::Running => return InfallibleStatus::Running,
                                InfallibleStatus::Success => {
                                    self.index += 1;
                                }
                            }
                        )+
                        _ => {
                            self.index = 0;
                            return InfallibleStatus::Success;
                        }
                    }
                }
            }
        }
        impl<C1, $($name,)+> FallibleBehavior<C1> for Sequence<($($name,)+)>
        where
            $($name: FallibleBehavior<C1>,)+
        {
            fn run_fallible(&mut self, blackboard: &mut C1) -> FallibleStatus {
                self.index = 0;
                self.body.0.run_fallible(blackboard)
            }
        }
        impl<C1, $($name,)+> EternalBehavior<C1> for Sequence<($($name,)+)>
        where
            $($name: EternalBehavior<C1>,)+
        {
            fn run_eternal(&mut self, blackboard: &mut C1) -> EternalStatus {
                self.index = 0;
                self.body.0.run_eternal(blackboard)
            }
        }

        impl<$($name,)+> IntoRon for Sequence<($($name,)+)>
        where
            $($name: IntoRon,)+
        {
            fn into_ron(&self) -> ron::Value {
                ron::Value::Map(
                    [
                        (ron::Value::String("sequence".to_string()), ron::Value::Seq(
                            vec![
                                $(
                                    self.body.$num.into_ron(),
                                )+
                            ].into_iter().collect()
                        ))
                    ].into_iter().collect()
                )
            }
        }
    }
}

impl_seq!(1 A 0);
impl_seq!(2 A 0 B 1);
impl_seq!(3 A 0 B 1 C 2);
impl_seq!(4 A 0 B 1 C 2 D 3);
impl_seq!(5 A 0 B 1 C 2 D 3 E 4);
impl_seq!(6 A 0 B 1 C 2 D 3 E 4 F 5);
impl_seq!(7 A 0 B 1 C 2 D 3 E 4 F 5 G 6);

impl<A> Sequence<A> {
    pub fn new(body: A) -> Self {
        Self { body, index: 0 }
    }
}

pub struct Select<A> {
    pub body: A,
    index: usize,
}

macro_rules! impl_sel {
    ($len: literal $($name: ident $num: tt)+) => {
        impl<C1, $($name,)+> Behavior<C1> for Select<($($name,)+)>
        where
            $($name: Behavior<C1>,)+
        {
            fn run(&mut self, blackboard: &mut C1) -> Status {
                loop {
                    match self.index {
                        $(
                            $num => match self.body.$num.run(blackboard) {
                                Status::Running => return Status::Running,
                                Status::Success => {
                                    self.index = 0;
                                    return Status::Success;
                                }
                                Status::Failure => {
                                    self.index += 1;
                                }
                            }
                        )+
                        _ => {
                            self.index = 0;
                            return Status::Failure;
                        }
                    }
                }
            }
        }
        impl<$($name,)+> CancelSafe for Select<($($name,)+)>
        where
            $($name: CancelSafe,)+
        {
            fn reset(&mut self) {
                self.index = 0;
                $(
                    self.body.$num.reset();
                )+
            }
        }
        impl<C1, $($name,)+> InfallibleBehavior<C1> for Select<($($name,)+)>
        where
            $($name: InfallibleBehavior<C1>,)+
        {
            fn run_infallible(&mut self, blackboard: &mut C1) -> InfallibleStatus {
                self.index = 0;
                self.body.0.run_infallible(blackboard)
            }
        }
        impl<C1, $($name,)+> FallibleBehavior<C1> for Select<($($name,)+)>
        where
            $($name: FallibleBehavior<C1>,)+
        {
            fn run_fallible(&mut self, blackboard: &mut C1) -> FallibleStatus {
                loop {
                    match self.index {
                        $(
                            $num => match self.body.$num.run_fallible(blackboard) {
                                FallibleStatus::Running => return FallibleStatus::Running,
                                FallibleStatus::Failure => {
                                    self.index += 1;
                                }
                            }
                        )+
                        _ => {
                            self.index = 0;
                            return FallibleStatus::Failure;
                        }
                    }
                }
            }
        }
        impl<C1, $($name,)+> EternalBehavior<C1> for Select<($($name,)+)>
        where
            $($name: EternalBehavior<C1>,)+
        {
            fn run_eternal(&mut self, blackboard: &mut C1) -> EternalStatus {
                self.index = 0;
                self.body.0.run_eternal(blackboard)
            }
        }

        impl<$($name,)+> IntoRon for Select<($($name,)+)>
        where
            $($name: IntoRon,)+
        {
            fn into_ron(&self) -> ron::Value {
                ron::Value::Map(
                    [
                        (ron::Value::String("select".to_string()), ron::Value::Seq(
                            vec![
                                $(
                                    self.body.$num.into_ron(),
                                )+
                            ].into_iter().collect()
                        ))
                    ].into_iter().collect()
                )
            }
        }
    }
}

impl_sel!(1 A 0);
impl_sel!(2 A 0 B 1);
impl_sel!(3 A 0 B 1 C 2);
impl_sel!(4 A 0 B 1 C 2 D 3);
impl_sel!(5 A 0 B 1 C 2 D 3 E 4);
impl_sel!(6 A 0 B 1 C 2 D 3 E 4 F 5);
impl_sel!(7 A 0 B 1 C 2 D 3 E 4 F 5 G 6);

impl<A> Select<A> {
    pub fn new(body: A) -> Self {
        Self { body, index: 0 }
    }
}

pub struct ParallelSequence<A> {
    pub body: A,
    index: usize,
    succeeded: usize,
}

macro_rules! impl_seq {
    ($len: literal $($name: ident $num: tt)+) => {
        impl<C1, $($name,)+> Behavior<C1> for ParallelSequence<($($name,)+)>
        where
            $($name: Behavior<C1>,)+
            Self: CancelSafe
        {
            fn run(&mut self, blackboard: &mut C1) -> Status {
                loop {
                    match self.index {
                        $(
                            $num => match self.body.$num.run(blackboard) {
                                Status::Running => {
                                    self.index += 1;
                                    return Status::Running;
                                }
                                Status::Success => {
                                    self.index += 1;
                                    self.succeeded += 1;
                                    if self.succeeded == $len {
                                        self.index = 0;
                                        self.succeeded = 0;
                                        return Status::Success;
                                    }
                                }
                                Status::Failure => {
                                    self.reset();
                                    return Status::Failure;
                                }
                            }
                        )+
                        _ => {
                            self.index = 0;
                        }
                    }
                }
            }
        }
        impl<$($name,)+> CancelSafe for ParallelSequence<($($name,)+)>
        where
            $($name: CancelSafe,)+
        {
            fn reset(&mut self) {
                self.index = 0;
                self.succeeded = 0;
                $(
                    self.body.$num.reset();
                )+
            }
        }
        impl<C1, $($name,)+> InfallibleBehavior<C1> for ParallelSequence<($($name,)+)>
        where
            $($name: InfallibleBehavior<C1>,)+
        {
            fn run_infallible(&mut self, blackboard: &mut C1) -> InfallibleStatus {
                loop {
                    match self.index {
                        $(
                            $num => match self.body.$num.run_infallible(blackboard) {
                                InfallibleStatus::Running => {
                                    self.index += 1;
                                    return InfallibleStatus::Running;
                                }
                                InfallibleStatus::Success => {
                                    self.index += 1;
                                    self.succeeded += 1;
                                    if self.succeeded == $len {
                                        self.index = 0;
                                        self.succeeded = 0;
                                        return InfallibleStatus::Success;
                                    }
                                }
                            }
                        )+
                        _ => {
                            self.index = 0;
                        }
                    }
                }
            }
        }
        impl<C1, $($name,)+> FallibleBehavior<C1> for ParallelSequence<($($name,)+)>
        where
            $($name: FallibleBehavior<C1>,)+
            Self: CancelSafe
        {
            fn run_fallible(&mut self, blackboard: &mut C1) -> FallibleStatus {
                loop {
                    match self.index {
                        $(
                            $num => match self.body.$num.run_fallible(blackboard) {
                                FallibleStatus::Running => {
                                    self.index += 1;
                                    return FallibleStatus::Running;
                                }
                                FallibleStatus::Failure => {
                                    self.reset();
                                    return FallibleStatus::Failure;
                                }
                            }
                        )+
                        _ => {
                            self.index = 0;
                        }
                    }
                }
            }
        }
        impl<C1, $($name,)+> EternalBehavior<C1> for ParallelSequence<($($name,)+)>
        where
            $($name: EternalBehavior<C1>,)+
        {
            fn run_eternal(&mut self, blackboard: &mut C1) -> EternalStatus {
                loop {
                    match self.index {
                        $(
                            $num => match self.body.$num.run_eternal(blackboard) {
                                t => {
                                    self.index += 1;
                                    return t;
                                }
                            }
                        )+
                        _ => {
                            self.index = 0;
                        }
                    }
                }
            }
        }

        impl<$($name,)+> IntoRon for ParallelSequence<($($name,)+)>
        where
            $($name: IntoRon,)+
        {
            fn into_ron(&self) -> ron::Value {
                ron::Value::Map(
                    [
                        (ron::Value::String("sequence".to_string()), ron::Value::Seq(
                            vec![
                                $(
                                    self.body.$num.into_ron(),
                                )+
                            ].into_iter().collect()
                        ))
                    ].into_iter().collect()
                )
            }
        }
    }
}

impl_seq!(1 A 0);
impl_seq!(2 A 0 B 1);
impl_seq!(3 A 0 B 1 C 2);
impl_seq!(4 A 0 B 1 C 2 D 3);
impl_seq!(5 A 0 B 1 C 2 D 3 E 4);
impl_seq!(6 A 0 B 1 C 2 D 3 E 4 F 5);
impl_seq!(7 A 0 B 1 C 2 D 3 E 4 F 5 G 6);

impl<A> ParallelSequence<A> {
    pub fn new(body: A) -> Self {
        Self {
            body,
            index: 0,
            succeeded: 0,
        }
    }
}

pub struct ParallelSelect<A> {
    pub body: A,
    index: usize,
    failed: usize,
}

macro_rules! impl_sel {
    ($len: literal $($name: ident $num: tt)+) => {
        impl<C1, $($name,)+> Behavior<C1> for ParallelSelect<($($name,)+)>
        where
            $($name: Behavior<C1>,)+
            Self: CancelSafe
        {
            fn run(&mut self, blackboard: &mut C1) -> Status {
                loop {
                    match self.index {
                        $(
                            $num => match self.body.$num.run(blackboard) {
                                Status::Running => {
                                    self.index += 1;
                                    return Status::Running;
                                },
                                Status::Success => {
                                    self.reset();
                                    return Status::Success;
                                }
                                Status::Failure => {
                                    self.index += 1;
                                    self.failed += 1;
                                    if self.failed == $len {
                                        self.index = 0;
                                        self.failed = 0;
                                        return Status::Failure;
                                    }
                                }
                            }
                        )+
                        _ => {
                            self.index = 0;
                        }
                    }
                }
            }
        }
        impl<$($name,)+> CancelSafe for ParallelSelect<($($name,)+)>
        where
            $($name: CancelSafe,)+
        {
            fn reset(&mut self) {
                self.index = 0;
                self.failed = 0;
                $(
                    self.body.$num.reset();
                )+
            }
        }
        impl<C1, $($name,)+> InfallibleBehavior<C1> for ParallelSelect<($($name,)+)>
        where
            $($name: InfallibleBehavior<C1>,)+
            Self: CancelSafe
        {
            fn run_infallible(&mut self, blackboard: &mut C1) -> InfallibleStatus {
                loop {
                    match self.index {
                        $(
                            $num => match self.body.$num.run_infallible(blackboard) {
                                InfallibleStatus::Running => {
                                    self.index += 1;
                                    return InfallibleStatus::Running;
                                },
                                InfallibleStatus::Success => {
                                    self.reset();
                                    return InfallibleStatus::Success;
                                }
                            }
                        )+
                        _ => {
                            self.index = 0;
                        }
                    }
                }
            }
        }
        impl<C1, $($name,)+> FallibleBehavior<C1> for ParallelSelect<($($name,)+)>
        where
            $($name: FallibleBehavior<C1>,)+
        {
            fn run_fallible(&mut self, blackboard: &mut C1) -> FallibleStatus {
                loop {
                    match self.index {
                        $(
                            $num => match self.body.$num.run_fallible(blackboard) {
                                FallibleStatus::Running => {
                                    self.index += 1;
                                    return FallibleStatus::Running;
                                },
                                FallibleStatus::Failure => {
                                    self.index += 1;
                                    self.failed += 1;
                                    if self.failed == $len {
                                        self.index = 0;
                                        self.failed = 0;
                                        return FallibleStatus::Failure;
                                    }
                                }
                            }
                        )+
                        _ => {
                            self.index = 0;
                        }
                    }
                }
            }
        }
        impl<C1, $($name,)+> EternalBehavior<C1> for ParallelSelect<($($name,)+)>
        where
            $($name: EternalBehavior<C1>,)+
        {
            fn run_eternal(&mut self, blackboard: &mut C1) -> EternalStatus {
                loop {
                    match self.index {
                        $(
                            $num => match self.body.$num.run_eternal(blackboard) {
                                t => {
                                    self.index += 1;
                                    return t;
                                }
                            }
                        )+
                        _ => {
                            self.index = 0;
                        }
                    }
                }
            }
        }

        impl<$($name,)+> IntoRon for ParallelSelect<($($name,)+)>
        where
            $($name: IntoRon,)+
        {
            fn into_ron(&self) -> ron::Value {
                ron::Value::Map(
                    [
                        (ron::Value::String("select".to_string()), ron::Value::Seq(
                            vec![
                                $(
                                    self.body.$num.into_ron(),
                                )+
                            ].into_iter().collect()
                        ))
                    ].into_iter().collect()
                )
            }
        }
    }
}

impl_sel!(1 A 0);
impl_sel!(2 A 0 B 1);
impl_sel!(3 A 0 B 1 C 2);
impl_sel!(4 A 0 B 1 C 2 D 3);
impl_sel!(5 A 0 B 1 C 2 D 3 E 4);
impl_sel!(6 A 0 B 1 C 2 D 3 E 4 F 5);
impl_sel!(7 A 0 B 1 C 2 D 3 E 4 F 5 G 6);

impl<A> ParallelSelect<A> {
    pub fn new(body: A) -> Self {
        Self {
            body,
            index: 0,
            failed: 0,
        }
    }
}

pub struct ParallelAny<A> {
    pub body: A,
    index: usize,
}

macro_rules! impl_sel {
    ($len: literal $($name: ident $num: tt)+) => {
        impl<C1, $($name,)+> Behavior<C1> for ParallelAny<($($name,)+)>
        where
            $($name: Behavior<C1>,)+
            Self: CancelSafe
        {
            fn run(&mut self, blackboard: &mut C1) -> Status {
                loop {
                    match self.index {
                        $(
                            $num => match self.body.$num.run(blackboard) {
                                Status::Running => {
                                    self.index += 1;
                                    return Status::Running;
                                },
                                Status::Success => {
                                    self.reset();
                                    return Status::Success;
                                }
                                Status::Failure => {
                                    self.reset();
                                    return Status::Failure;
                                }
                            }
                        )+
                        _ => {
                            self.index = 0;
                        }
                    }
                }
            }
        }
        impl<$($name,)+> CancelSafe for ParallelAny<($($name,)+)>
        where
            $($name: CancelSafe,)+
        {
            fn reset(&mut self) {
                self.index = 0;
                $(
                    self.body.$num.reset();
                )+
            }
        }
        impl<C1, $($name,)+> InfallibleBehavior<C1> for ParallelAny<($($name,)+)>
        where
            $($name: InfallibleBehavior<C1>,)+
            Self: CancelSafe
        {
            fn run_infallible(&mut self, blackboard: &mut C1) -> InfallibleStatus {
                loop {
                    match self.index {
                        $(
                            $num => match self.body.$num.run_infallible(blackboard) {
                                InfallibleStatus::Running => {
                                    self.index += 1;
                                    return InfallibleStatus::Running;
                                },
                                InfallibleStatus::Success => {
                                    self.reset();
                                    return InfallibleStatus::Success;
                                }
                            }
                        )+
                        _ => {
                            self.index = 0;
                        }
                    }
                }
            }
        }
        impl<C1, $($name,)+> FallibleBehavior<C1> for ParallelAny<($($name,)+)>
        where
            $($name: FallibleBehavior<C1>,)+
            Self: CancelSafe
        {
            fn run_fallible(&mut self, blackboard: &mut C1) -> FallibleStatus {
                loop {
                    match self.index {
                        $(
                            $num => match self.body.$num.run_fallible(blackboard) {
                                FallibleStatus::Running => {
                                    self.index += 1;
                                    return FallibleStatus::Running;
                                },
                                FallibleStatus::Failure => {
                                    self.reset();
                                    return FallibleStatus::Failure;
                                }
                            }
                        )+
                        _ => {
                            self.index = 0;
                        }
                    }
                }
            }
        }
        impl<C1, $($name,)+> EternalBehavior<C1> for ParallelAny<($($name,)+)>
        where
            $($name: EternalBehavior<C1>,)+
        {
            fn run_eternal(&mut self, blackboard: &mut C1) -> EternalStatus {
                loop {
                    match self.index {
                        $(
                            $num => match self.body.$num.run_eternal(blackboard) {
                                t => {
                                    self.index += 1;
                                    return t;
                                }
                            }
                        )+
                        _ => {
                            self.index = 0;
                        }
                    }
                }
            }
        }

        impl<$($name,)+> IntoRon for ParallelAny<($($name,)+)>
        where
            $($name: IntoRon,)+
        {
            fn into_ron(&self) -> ron::Value {
                ron::Value::Map(
                    [
                        (ron::Value::String("select".to_string()), ron::Value::Seq(
                            vec![
                                $(
                                    self.body.$num.into_ron(),
                                )+
                            ].into_iter().collect()
                        ))
                    ].into_iter().collect()
                )
            }
        }
    }
}

impl_sel!(1 A 0);
impl_sel!(2 A 0 B 1);
impl_sel!(3 A 0 B 1 C 2);

impl<A> ParallelAny<A> {
    pub fn new(body: A) -> Self {
        Self { body, index: 0 }
    }
}
