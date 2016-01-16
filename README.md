GradientShop
===================================
This is a course project on Computer Graphics.

Implementation of paper [GradientShop: A Gradient-Domain Optimization Framework
for Image and Video Filtering](http://grail.cs.washington.edu/projects/gradientshop/demos/gs_paper_TOG_2009.pdf)

The paper has provided GradientShop C++ Framework API, thus I haven't compile it through on neither OS X nor windows. So this is my own implementation, which is based on OpenCV C++ API. Core concept in this project:

- basics: gradient magnitude/orientation
- steerable filter
- local edge length estimation
- saliency
- conjugate gradient (solve quadratic problem)


Applications implemented:

- Saliency Shappening
