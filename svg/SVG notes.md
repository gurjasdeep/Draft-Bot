Why an SVG? Why not?

- SVG is Scalable Vector graphics, instead of per pixel data, it uses a set of commands for drawings.

- SVG is in XML
each XML tag in SVG is used to create a line/shape.

for example, the tag

 `<path>`

it has a main attribute 'd'
inside d is all the data you would need to create a path (a continous custom line) 

example:


`<svg height="210" width="400" xmlns="http://www.w3.org/2000/svg">
  <path d="M150 5 L75 200 L225 200 Z"
  style="fill:none;stroke:green;stroke-width:3" />
</svg>
`


M - move to
L - line to
Z - close

so what this does is, move to position 150, 5
then make a line from there to 75, 200; then 225, 200 then close path (back to 150, 5)

what we are doing in code? we are reading this d attribute, converting 
it to python objects, which we can reuse.

Everytime the Move line command comes, we make a new path object, every path object in code is what we have to draw on paper, this 
fixes 
the issue where if the 
svg path isnt continous and moves to a point in between without making a line, the bot would read it as making a line.
so now instead we are just making a new path object for each continous stroke defined in 
d attribute of 
path tag.


## 5/10/26:
- vector_paths.py
  - can convert svg files with path, rect, circle and a few other similar tag to paths for bot
  - does not support tranform tag (major setback apparantly)
