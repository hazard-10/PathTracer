## Path Tracer

This is a functional global illumination renderer that supports lambertian, mirror, refraction, and glass materials for photo-realistic rendering. I also used bounding volume hierarchy (BVH) to accelerate ray-object collision detection.

    ![path tracing demo](../../renders/output_refraction_high_res_crop.png)

### Features

* **Lambertian Shading**: Diffuse shading that simulates the scattering of light in many directions.
* **Mirror Reflection**: Creates perfect mirror-like reflections.
* **Refraction**: Simulates the bending of light as it passes through transparent materials.
* **Glass Materials**: Renders transparent objects with both reflection and refraction.
* **Bounding Volume Hierarchy (BVH)**: Accelerates the ray-object collision detection by organizing objects into a tree structure.
