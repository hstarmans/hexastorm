# Project Management

This section covers the following topics:
* Setting up a folder structure
* using git for version control
* publishing your project on github
* Choosing a license

Time for some hands-on practice!

We have 3 short videos and accompanying exercises for you to go through one by one.
Watch the video first, then apply the exercises to your own project.
Use the [slides]("../slides/slides_project-setup.html") linked above to follow links discussed in the video.
Be sure to ask for help when you need it!

## Project setup

### Video
<iframe src="https://player.vimeo.com/video/462773031" width="100%" height="400px">
</iframe>
<!-- ```{r}
vembedr::embed_url("https://vimeo.com/462773031")
``` -->

### Slides
<iframe src="../slides/slides_project-setup.html#3" width="100%" height="400px">
</iframe>
<!-- ```{r}
knitr::include_url("../slides/slides_project-setup.html#3")
``` -->

### Exercises

_NB: You can check the slides for more detail._
- Use cookiecutter to install a project structure in your system, following the video or [these slides](../slides/slides_project-setup.html#4).
- If you have trouble with cookiecutter: use the instructions [on this slide](../slides/slides_project-setup.html#6) instead.
- Take a look at the folder and files within it, to see where your answers to cookiecutter ended up!
- Place your project files in the right folder.
- Adjust paths in your code, and be sure to use relative paths!
- Does your code run in the new folder structure?

## Version control

### Video
<iframe src="https://player.vimeo.com/video/463264170" width="100%" height="400px">
</iframe>
<!-- ```{r}
vembedr::embed_url("https://vimeo.com/463264170")
``` -->

### Slides
<iframe src="../slides/slides_project-setup.html#15" width="100%" height="400px">
</iframe>
<!-- ```{r}
knitr::include_url("../slides/slides_project-setup.html#15")
``` -->

### Exercise

- Follow the steps in the video (or [on these slides](../slides/slides_project-setup.html#17)) to turn your folder into a git repository
- Make a remote version of your project on GitHub!
- Please note: are there (temporary) files you do not wish to track?
  Add them to the `.gitignore` file.
  Consider a `.gitignore` template for your language: examples on [this github repo](https://github.com/github/gitignore).
- Can you use Git and push to Github from your IDE?
- Experiment with editing and committing on github itself.
  You can then 'download' your code to your local repository using `git pull`.
- Optional: What happens if you edit the same file online and locally, and try to push/pull?
  (Hint: this often causes a 'merge conflict', which is no fun to experience.
  Going through it today means we can assist you if necessary!)

## Publication & licensing

### Video
<iframe src="https://player.vimeo.com/video/463659936" width="100%" height="400px">
</iframe>
<!-- ```{r}
vembedr::embed_url("https://vimeo.com/463659936")
``` -->

### Slides
<iframe src="../slides/slides_project-setup.html#9" width="100%" height="400px">
</iframe>
<!-- ```{r}
knitr::include_url("../slides/slides_project-setup.html#9")
``` -->

### Exercise

- Check the license in your project
- Take a look at other license options via [choosealicense.com](https://choosealicense.com/).
- Do you want to change your license? Go into your Github and change the LICENSE.md file. Don't forget to pull your remote changes!