# Unintuitive Optimization For Performing Paths Union

<svg width="100%" viewBox="0 0 600 245">
  <rect x="0" y="0" width="600" height="245" fill="white"></rect>
  <path d="M56 237 L130 9 L204 237 L10 96 H250Z" fill="black"></path>
  <path d="M90 90 A75 75 0 1 0 240 90 A75 75 0 0 0 90 90Z" fill="oklch(57.35% 0.2072 19.06)" fill-opacity="0.8"></path>
  <path d="M260 154 v-16 h40 v-8 l16 16 l-16 16 v-8z" fill="oklch(76.47% 0.1825 114.35)"></path>
  <path d="m 430 9 l -5.9785 18.4199 a 75 75 0 0 0 -34.0215 62.5801 a 75 75 0 0 0 0.4492 6 l -80.4492 0 l 74.248 53.9648 l -28.248 87.0352 l 74 -53.7832 l 74 53.7832 l -23.9004 -73.6406 a 75 75 0 0 0 58.3242 -58.9453 l 11.5762 -8.4141 l -10.4492 0 a 75 75 0 0 0 0.4492 -6 a 75 75 0 0 0 -105.834 -68.1621 l -4.166 -12.8379 z" stroke="black" fill="transparent" stroke-width="4" stroke-linecap="round" stroke-linejoin="round"></path>
</svg>

This is a standalone example of how performing union on a lot of paths can be done quickly. This was part of a real-world task I had to solve at one of my previous jobs.

The test case is `Flag.svg`. Computing the union of all paths is done using Skia's PathOps. Running `python main.py` will output the final contour to `Contour.svg`.

Inside `renderer` I have made a very basic vector renderer that can take the same SVG as the input, and draw it to an image. While it's doing that it will also sort the paths by their overlap. The path that overlaps the most with other paths will be placed first. This was used to experiment with strategies for achieving higher performance, which ultimately failed.

To run the renderer, just `cargo run`.
