#[cxx_qt::bridge]
mod corsairust_rgb_controller {

    unsafe extern "C++" {
        include!("OpenRGB/OpenRGBPluginInterface.h");
        type OpenRGBPluginInterface;
    }

    #[cxx_qt::qobject]
    pub struct CorsairustRGBController {
        #[qproperty]
        number: i32,
        #[qproperty]
        string: i32,
    }

    impl Default for CorsairustRGBController {
        fn default() -> Self {
            Self {
                number: 0,
                string: 0,
            }
        }
    }

    impl qobject::CorsairustRGBController {
        #[qinvokable]
        pub fn increment_number(self: Pin<&mut Self>) {
            let previous = *self.as_ref().number();
            self.set_number(previous + 1);
        }

        #[qinvokable]
        pub fn say_hi(&self, string: i32, number: i32) {
            println!(
                "Hi from Rust! String is '{}' and number is {}",
                string, number
            );
        }
    }
}