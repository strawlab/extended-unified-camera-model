#[cfg(feature = "serde-serialize")]
mod serde_tests {
    use nalgebra::RealField;

    use cam_geom::intrinsic_test_utils::roundtrip_intrinsics;
    use extended_unified_camera_model::EucmParams;

    fn check_roundtrip<R: RealField + serde::de::DeserializeOwned>(eps: R) {
        let buf = include_str!("eucm-cal.json");
        let eucam: EucmParams<R> = serde_json::from_str(buf).unwrap();

        roundtrip_intrinsics(&eucam, 1920, 1080, 5, 65, nalgebra::convert(eps));
    }

    #[test]
    fn roundtrip_f32() {
        check_roundtrip::<f32>(1e-3f32);
    }

    #[test]
    fn roundtrip_f64() {
        check_roundtrip::<f64>(1e-12);
    }
}
