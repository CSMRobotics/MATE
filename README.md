This project follows the standard modern cmake project structure. (except for having no /apps/ folder because that is stupid) 
A bit about formatting:
- Ensure all includes are absolute and not relative (helps ensure that you actually import the library/header you want)
- Keep all .h files in include/ (yes this is a double edged sword but it will help with keeping the number of CMakeLists.txt down)
- Place all external librarys in libs/ (make the library inside a subdirectory e.g. libs/libcv2/) (dont forget to link library in ./CMakeLists.txt)