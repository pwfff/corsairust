use cxx_qt_build::CxxQtBuilder;

fn main() {
    CxxQtBuilder::new().file("rust/src/cxxqt_object.rs").build();
}