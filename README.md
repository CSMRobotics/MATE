This project follows the standard modern cmake project structure.  
A bit about formatting:
- Ensure all includes are absolute and not relative (helps ensure that you actually import the library/header you want)
- Keep all .h files in include/ (yes this is a double edged sword but it will help with organization; feel free to make as many subfolders in include as you want)
- Place all external librarys in libs/ (make the library inside a subdirectory e.g. libs/libcv2/)