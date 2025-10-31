[![Crates.io](https://img.shields.io/crates/v/extended-unified-camera-model.svg)](https://crates.io/crates/extended-unified-camera-model)
[![Documentation](https://docs.rs/extended-unified-camera-model/badge.svg)](https://docs.rs/extended-unified-camera-model/)
[![Crate License](https://img.shields.io/crates/l/extended-unified-camera-model.svg)](https://crates.io/crates/extended-unified-camera-model)
[![Dependency status](https://deps.rs/repo/github/strawlab/extended-unified-camera-model/status.svg)](https://deps.rs/repo/github/strawlab/extended-unified-camera-model)

📷 📐 Extended Unified Camera Model

This crate provides an Extended Unified Camera Model. The crate is in pure Rust,
can be compiled in `no_std` mode, implements the
[`IntrinsicParameters`](https://docs.rs/cam-geom/latest/cam_geom/trait.IntrinsicParameters.html)
trait from the [`cam-geom`](https://crates.io/crates/cam-geom) and provides
support to read and write camera models using
[`serde`](https://crates.io/crates/serde) if compiled with the `serde-serialize`
feature.

The Extended Unified Camera Model was originally described in B. Khomutenko, G.
Garcia, and P. Martinet. "An enhanced unified camera model". *IEEE Robotics and
Automation Letters*, 1(1):137–144, Jan 2016. The formulation here follows the
formulation in sections 2.2 and 2.3 in the paper "The Double Sphere Camera
Model" by Vladyslav Usenko, Nikolaus Demmel and Daniel Cremers
[doi:10.1109/3DV.2018.00069](https://doi.org/10.1109/3DV.2018.00069).

## License

Licensed under either of these:

 * Apache License, Version 2.0, ([LICENSE-APACHE](LICENSE-APACHE) or
   <https://www.apache.org/licenses/LICENSE-2.0>)
 * MIT license ([LICENSE-MIT](LICENSE-MIT) or
   <https://opensource.org/licenses/MIT>)
