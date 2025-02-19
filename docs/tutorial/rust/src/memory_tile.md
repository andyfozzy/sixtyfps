# Memory Tile

With the skeleton in place, let's look at the first element of the game, the memory tile. It will be the
visual building block that consists of an underlying filled rectangle background, the icon image. Later we'll add a
covering rectangle that acts as a curtain. The background rectangle is declared to be 64 logical pixels wide and tall,
and it is filled with a soothing tone of blue. Note how lengths in the `.60` language have a unit, here
the `px` suffix. That makes the code easier to read and the compiler can detect when your are accidentally
mixing values with different units attached to them.

We copy the following code inside of the `sixtyfps!` macro:

```60
{{#include main_memory_tile.rs:tile}}
```

Inside the <span class="hljs-built_in">Rectangle</span> we place an <span class="hljs-built_in">Image</span> element that
loads an icon with the <span class="hljs-built_in">@image-url()</span> macro.
When using the `sixtyfps!` macro, the path is relative to the folder in which the Cargo.toml is located.
When using .60 files, it is relative to the folder of the .60 file containing it.
This icon and others we're going to use later need to be installed first. You can download a
[Zip archive](https://sixtyfps.io/blog/memory-game-tutorial/icons.zip) that we have prepared and extract it with the
following two commands:

```sh
curl -O https://sixtyfps.io/blog/memory-game-tutorial/icons.zip
unzip icons.zip
```

This should unpack an `icons` directory containing a bunch of icons.

Running the program with `cargo run` gives us a window on the screen that shows the icon of a bus on a
blue background.

![Screenshot of the first tile](https://sixtyfps.io/blog/memory-game-tutorial/memory-tile.png "Memory Tile Screenshot")
