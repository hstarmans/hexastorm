# Documentation

Time for some documentation around your code! 

The following topics are covered:
* commenting code and functions (docstrings)
* writing a README file

As usual, we have short videos and accompanying exercises. 
Watch the video, then apply the exercises to your own project. 
Use the slides linked above for reference, and be sure to ask for help when you need it!

## Comments and docstrings

### Video
<iframe src="https://player.vimeo.com/video/463992354" width="100%" height="400px">
</iframe>
<!-- ```{r}
vembedr::embed_url("https://vimeo.com/463992354")
``` -->

### Slides
<iframe src="../slides/slides_documentation.html#4" width="100%" height="400px">
</iframe>
<!-- ```{r}
knitr::include_url("../slides/slides_documentation.html#4")
``` -->

### Exercise
- Add a docstring to a function, preferably the last function you worked on (so it's fresh in your memory).
  _Keep in mind: what does my user need to know when they are working with this function?_
- Grab a limited chunk of code to work on, and look at the existing comments.
  Can you replace a 'how' comment with a 'why' comment?
  _Think: what is the purpose of this code? Rather than: this is how this code works._
- Are there elements in your chunk that are currently without comments that would benefit from clarification? 
  _Try to comment on the thought behind the code rather than simply translating its process in English._
- Can you delete superfluous comments or zombie code?

## The README page

### Video
<iframe src="https://player.vimeo.com/video/464027978" width="100%" height="400px">
</iframe>
<!-- ```{r}
vembedr::embed_url("https://vimeo.com/464027978")
``` -->

### Slides
<iframe src="../slides/slides_documentation.html#16" width="100%" height="400px">
</iframe>
<!-- ```{r}
knitr::include_url("../slides/slides_documentation.html#16")
``` -->


### Exercise
Edit your README file.
Take your time. This is important!

Make sure to include the following information:
- What does your project do?
- How does the user access your project? (E.g. download, or clone with `git clone`...)
- How does the user call the main script(s) that should be executed?
- And perhaps any other elements you are inspired to add (check out the examples!)

Do you want to truly understand the importance of decent installation information?
[Read this experience by Elisabeth Bik](https://twitter.com/MicrobiomDigest/status/1283082285097422848), trying to install image forensics software.
Look at the [README of the software](https://github.com/GuidoBartoli/sherloq), and see what you can learn.

_NB: We will have another chance to take a look at dependencies and prerequisites, but if you have opportunity to add these here: please do!_
