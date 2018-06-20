# Monte Carlo Path Tracer

![alt text](https://github.com/Esgrove/pathtracer/blob/master/head1.png)

A somewhat optimized C++ parallel CPU [path tracer](https://en.wikipedia.org/wiki/Path_tracing), programmed originally for the _Advanced Computer Graphics_ course at Aalto University, and then developed a bit further after the course. I was among the top 3 students on the course (spring 2017 edition), as I put in a lot of extra effort into the assignments. Note that I have included here only the relevant source code instead of the whole VS project (solution) together with the Nvidia graphics framework used. Some of the code is template starter code provided by the course, but all the main functionality is my own code, which includes the BVH implementation completely and all ray-tracing code. Of course, there remains a lot of room for improvement (more bells and whistles) and further optimization, especially in the form of SIMD.
The "head" scene used here for the two example renders is the [LPS head](http://casual-effects.com/data/), around which I added the walls using some open-source textures I found on the internet.

#### Main features

 - Efficient parallel rendering using all available CPU cores / threads (though no SIMD)
 - [Bounding volume hierarchy](https://en.wikipedia.org/wiki/Bounding_volume_hierarchy) (BVH) for accelerated ray tracing, using the Surface Area Heuristic (SAH) algorithm for deciding the split locations. The calculation of the bounding box surface area is implemented using dynamic programming principles, so that it can be done in linear time. The tree building is recursion-based and parallelized where it counts. After the BVH (binary) tree has been build, it is "flattened" from a pointer-based tree structure into a depth-first pre-order list, meaning the left child node is always next in the list, with each node storing only the index for the right child node. The size of each node is padded to be 64 bytes for good cache-line behavior. The BVH tree traversal is implemented with a custom stack instead of recursion, with optimized ray-node intersection checks and early exits for shadow rays.
 - Low-discrepancy sequence sampling for both AA and lights using the [Sobol sequence](https://en.wikipedia.org/wiki/Sobol_sequence)
 - Area light sources with support for multiple, individually adjustable lightsources. For each ray, the sampled lightsource is selected randomly with a bias towards lights with more intensity, implemented efficiently with std::valarray.
  - Depth of Field effect: Compute distance to the set "focal plane" from the camera origin, then randomly nudge the ray origin inside an "aperture" disk by mapping uniform random values with the Shirley-Chiu mapping (_A Low Distortion Map Between Disk and Square_). Finally, the new ray direction is calculated from this point. Bigger aperture gives a larger depth of field blur effect.
 - Russian roulette path termination for unbiased tracing results
 - Normalized Blinn-Phong shading (diffuse & specular)
 - Bilinear texture filtering
 - Tangent space normal mapping
 - Support for alpha textures
 - Gaussian AA filtering with variable filter size 
 - Two different rendering modes (main difference is in how AA samples are handeled)
 - Also includes a classical ray tracer supporting specular reflections, area lights and soft shadows.
 - Color saturation function for making the end results "pop" a bit more :)

 ![alt text](https://github.com/Esgrove/pathtracer/blob/master/head2.png)

 ##### Classical ray tracing with the depth-of-field effect

 ![alt text](https://github.com/Esgrove/pathtracer/blob/master/crytek_sponza_whitted_dof.png)