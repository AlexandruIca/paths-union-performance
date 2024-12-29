![Showcasing Union](https://github.com/user-attachments/assets/d70a7fc8-4271-4c64-83e7-c49a06b9f58b)

# Unintuitive Optimization For Performing Paths Union

This is a standalone example of how performing union on a lot of paths can be done quickly. It was part of a real-world task I had to solve at one of my previous jobs. It's also the source code for [this](https://minus-ze.ro/posts/unintuitive-optimization-for-performing-paths-union/) blog post.

The test case is `Flag.svg`. Computing the union of all paths is done using Skia's PathOps. Running `python main.py` will output the final contour to `Contour.svg`.

Inside `renderer` I have made a very basic vector renderer that can take the same SVG as the input, and draw it to an image. While it's doing that it will also sort the paths by their overlap. The path that overlaps the most with other paths will be placed first. This was used to experiment with strategies for achieving higher performance, which ultimately failed.

To run the renderer, just `cargo run`.
