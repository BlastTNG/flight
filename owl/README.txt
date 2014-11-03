Owl is a tool which can be used to display data from GetData in real-time.
It includes a drag-and-drop interface and a web front-end to share a
current Owl session.

COMPILING AND INSTALING
============================================================================
Short answer: run "qmake-qt4 && make". Then run "./owl" or copy to your
directory of choice.

You can change where owl will install to by editing owl.pro.

If you want web server support (more below) you must also cd into "owl-cgi",
edit owl-cgi.pro and run "qmake-qt4 && make" and copy it into your cgi-bin
directory.

SETTING UP OWL
============================================================================
An owl setup is composed of a number of boxes, each which contain a number
of data items.

During your setup you may find it useful to move of hide the dockers. You
can move them by clicking and dragging or hide them by clicking on the
respective toolbar option.

First in the configure box, select "Owl" from the combobox and enter in the
GetData DirFile path you want to use.

To create a new box drag "Box" from the "Insert" docker to where you want it.
You can then drag its corners to resize it or drag the title to move it
around. If you want to modify its settings later, click its title, or choose
it from the configure combobox

You can remove the box, change its title and style in the "Configure" docker.
More about styles below.

You can then drag data items (Numbers, Multis, Date/Times, Dirfile names)
into the boxes. In the "Configure" docker for any type of data item you can
remove it, change it's caption, caption style, data style (i.e., the style of
the data field) and in everything but the "Dirfile Name" type, source.
 - Numbers display a GetData source as a number. You can specify the exact
   format for the number by entering a printf string under format. As an
   extra feature, you can set the format to "%b" to see it as a binary field.
   It is undefined to request a type with more bytes than the source
   supports. For example, do not request a double for a float type. Numbers
   support extremas. See more below.
 - Multis implement a mapping between specific values and strings. Once a multi
   is created, in the "format" section of the "Configure" docker click
   "New Row". In the created row, type in the number you want to map on the
   left and the string that number represents of the right. For example,
   you may wish to map '0' (without quotes) to "Default" (without quotes).
   You can specify styles for a certain mapping by selecting the desired row
   and choosing a style. More on styles below.
 - Dates and times display a GetData ctime source as a date and/or time.
   You must specify a strftime string in format.
 - A "Dirfile name" simply displays the name of the current dirfile.

If you want to modify a data item later, just click on it.

STYLES
============================================================================
[No Style]    [f]            [b]             [B]    [i]
 Style name   Foreground clr Background clr  Bold   Italic

To set the font, foreground colour or background colour of an object, select
a style the style combobox, or choose "New Style" to create a new one. When
you change a certain style, all objects with that style will also change.

EXTREMAS
============================================================================
Extremas provide special formatting for number values that exceed or fall
short of a certain value. To set an extrema for a certain object, select
that object, and in the "Extrema" section of the "Configure" docker select
an extrema or choose "New Extrema" to create a new one. When you change
the parameters of an extrema, all objects with that extrema will also
change. Below is an explanation of the parameters:

 - XHigh: values above this amount will be displayed with the below style.
 - High:  values between this an XHigh will be displayed with the below
          style. If this value is equal to or above XHigh, it will be
          ignored.
 - XLow:  values below this amount will be displayed with the below style.
 - Low:   values between this and XLow will be displayed with the below
          style. If this value is equal to or below Low, it will be ignored.

WEB SERVER
============================================================================
For information on setting up the web-server click on the question mark in
the "Web Server" docker. A web-server can be used to share default values
with Cow, or to share a current session with anyone with a web browser.

IMPORTING AN EXISTING PALANTIR SETUP
============================================================================
To migrate from an existing palantir setup, you can click "Open" from the
toolbar and select a .pal file. The importer is designed to eliminate a lot
of typing, but does not import all data. Owl will place all palantir boxes
in the top left corner. Before modifying anything, save your session as an
owl file and reopen it. Then, you should check to make sure all styles and
data have been correctly imported.
