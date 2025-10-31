#![no_std]
#![doc = include_str!("../README.md")]
use nalgebra::{Dim, OMatrix, RealField, U2, U3};

#[cfg(feature = "serde-serialize")]
use serde::{Deserialize, Serialize};

/// Parameters of an Extended Unified Camera Model
///
/// This implements [cam_geom::IntrinsicParameters].
///
/// Extended Unified Camera Model originally described in B. Khomutenko, G.
/// Garcia, and P. Martinet. "An enhanced unified camera model". *IEEE Robotics
/// and Automation Letters*, 1(1):137–144, Jan 2016. The formulation here
/// follows the formulation in sections 2.2 and 2.3 in the paper "The Double
/// Sphere Camera Model" by Vladyslav Usenko, Nikolaus Demmel and Daniel Cremers
/// [doi:10.1109/3DV.2018.00069](https://doi.org/10.1109/3DV.2018.00069).
///
/// When compiled with the `serde-serialize` feature, can be serialized and
/// deserialized using serde.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct EucmParams<R> {
    pub fx: R,
    pub fy: R,
    pub cx: R,
    pub cy: R,
    pub alpha: R,
    pub beta: R,
}

impl<R: RealField> cam_geom::IntrinsicParameters<R> for EucmParams<R> {
    type BundleType = cam_geom::ray_bundle_types::SharedOriginRayBundle<R>;

    fn pixel_to_camera<IN, NPTS>(
        &self,
        pixels: &cam_geom::Pixels<R, NPTS, IN>,
    ) -> cam_geom::RayBundle<
        cam_geom::coordinate_system::CameraFrame,
        Self::BundleType,
        R,
        NPTS,
        nalgebra::Owned<R, NPTS, nalgebra::U3>,
    >
    where
        Self::BundleType: cam_geom::Bundle<R>,
        IN: nalgebra::Storage<R, NPTS, nalgebra::U2>,
        NPTS: nalgebra::Dim,
        nalgebra::DefaultAllocator: nalgebra::allocator::Allocator<nalgebra::U1, nalgebra::U2>,
        nalgebra::DefaultAllocator: nalgebra::allocator::Allocator<NPTS, nalgebra::U2>,
        nalgebra::DefaultAllocator: nalgebra::allocator::Allocator<NPTS, nalgebra::U3>,
    {
        // Implementation of Usenko et al. equations 18-22.
        let mut result = cam_geom::RayBundle::new_shared_zero_origin(OMatrix::zeros_generic(
            NPTS::from_usize(pixels.data.nrows()),
            U3::from_usize(3),
        ));

        let one: R = nalgebra::convert(1.0);
        let two: R = nalgebra::convert(2.0);

        for i in 0..pixels.data.nrows() {
            let u = pixels.data[(i, 0)].clone();
            let v = pixels.data[(i, 1)].clone();
            let mx = (u - self.cx.clone()) / self.fx.clone();
            let my = (v - self.cy.clone()) / self.fy.clone();
            let r2 = mx.clone() * mx.clone() + my.clone() * my.clone();
            let mz_num = one.clone() - self.beta.clone() * self.alpha.clone().powi(2) * r2.clone();
            let mz_denom = self.alpha.clone()
                * (one.clone()
                    - (two.clone() * self.alpha.clone() - one.clone())
                        * self.beta.clone()
                        * r2.clone())
                .sqrt()
                + (one.clone() - self.alpha.clone());
            let mz = mz_num.clone() / mz_denom.clone();
            let norm =
                one.clone() / (mx.clone().powi(2) + my.clone().powi(2) + mz.clone().powi(2)).sqrt();
            let x = mx.clone() * norm.clone();
            let y = my.clone() * norm.clone();
            let z = mz * norm;
            result.data[(i, 0)] = x;
            result.data[(i, 1)] = y;
            result.data[(i, 2)] = z;
        }

        result
    }

    fn camera_to_pixel<IN, NPTS>(
        &self,
        camera: &cam_geom::Points<cam_geom::coordinate_system::CameraFrame, R, NPTS, IN>,
    ) -> cam_geom::Pixels<R, NPTS, nalgebra::Owned<R, NPTS, nalgebra::U2>>
    where
        IN: nalgebra::Storage<R, NPTS, nalgebra::U3>,
        NPTS: nalgebra::Dim,
        nalgebra::DefaultAllocator: nalgebra::allocator::Allocator<NPTS, nalgebra::U2>,
    {
        let one: R = nalgebra::convert(1.0);

        // Implementation of Usenko et al. equations 16 and 17.
        let mut result = cam_geom::Pixels {
            data: OMatrix::zeros_generic(NPTS::from_usize(camera.data.nrows()), U2::from_usize(2)),
        };

        for i in 0..camera.data.nrows() {
            let x = camera.data[(i, 0)].clone();
            let y = camera.data[(i, 1)].clone();
            let z = camera.data[(i, 2)].clone();

            let d = (self.beta.clone() * (x.clone() * x.clone() + y.clone() * y.clone())
                + z.clone() * z.clone())
            .sqrt();
            let denom =
                self.alpha.clone() * d.clone() + (one.clone() - self.alpha.clone()) * z.clone();
            let u = self.fx.clone() * (x.clone() / denom.clone()) + self.cx.clone();
            let v = self.fy.clone() * (y / denom) + self.cy.clone();

            result.data[(i, 0)] = u;
            result.data[(i, 1)] = v;
        }
        result
    }
}
