# Promenad
An slice of life adventure in procedural animation and possibly some AI. Specialization course at TGA (The Game Assembly).

## How to build and run

Full rebuild:

```bash
make clean all
```

Run application (while developing):

```bash
make run
```

Run test suite:

```bash
make check
```

[![Build the darn thing](https://github.com/jordgubben/promenad/actions/workflows/build.yml/badge.svg)](https://github.com/jordgubben/promenad/actions/workflows/build.yml)


## Dependencies

 - [Catch 2](https://github.com/catchorg/Catch2/)
 - [Raylib](https://www.raylib.com/)

## Folders

  - `src/` – Our code.
  - `res/` – Models, textures, config-files etc.
  - `bin/` – Where the executables go.
  - `tmp/` – Build intermediates, logs etc.
