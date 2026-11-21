# Third-party notices

MMSim itself is MIT licensed (see `LICENSE`). It includes or links the components below.
Full texts of the longer licences are in the `licenses/` folder.

| Component | Version | Licence | Copyright |
|---|---|---|---|
| SDL2 (`SDL2.dll`) | 2.32.x | zlib | Copyright (C) 1997-2025 Sam Lantinga |
| Dear ImGui (docking branch) | 1.93.0 | MIT | Copyright (c) 2014-2026 Omar Cornut |
| ImPlot | 1.1 | MIT | Copyright (c) 2020 Evan Pezent |
| nlohmann/json | 3.12.0 | MIT | Copyright (c) 2013-2025 Niels Lohmann |
| stb_image_write | pinned commit | MIT or public domain | Sean Barrett and contributors |
| doctest (tests only, not shipped) | 2.5.3 | MIT | Copyright (c) 2016-2023 Viktor Kirilov |
| Roboto-Medium (UI font, `fonts/`) | as in Dear ImGui `misc/fonts` | Apache-2.0 | Copyright 2011 Google Inc. |
| Cousine-Regular (number font, `fonts/`) | as in Dear ImGui `misc/fonts` | SIL OFL 1.1 | Digitized data copyright (c) 2010 Google Corporation |
| GCC runtime (libgcc, libstdc++), linked statically | MSYS2 UCRT64 | GPL-3.0 with the GCC Runtime Library Exception | Free Software Foundation |
| winpthreads, linked statically | MSYS2 UCRT64 | MIT / ZPL | MinGW-w64 project |

The maze files of the `mazefiles` collection are **not** included in this package or in the
repository. Fetch them yourself with `tools/fetch_mazes.ps1` and read the licence notes of that
collection first. The test mazes in `mazes/test/` are original to this project (MIT).

## MIT licence (Dear ImGui, ImPlot, nlohmann/json, stb, doctest)

Permission is hereby granted, free of charge, to any person obtaining a copy of this software
and associated documentation files (the "Software"), to deal in the Software without
restriction, including without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

## zlib licence (SDL2)

Copyright (C) 1997-2025 Sam Lantinga <slouken@libsdl.org>

This software is provided 'as-is', without any express or implied warranty. In no event will
the authors be held liable for any damages arising from the use of this software.

Permission is granted to anyone to use this software for any purpose, including commercial
applications, and to alter it and redistribute it freely, subject to the following
restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote
   the original software. If you use this software in a product, an acknowledgment in the
   product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as
   being the original software.
3. This notice may not be removed or altered from any source distribution.

## Fonts

- Roboto-Medium: Apache License 2.0, see `licenses/Apache-2.0.txt` and `licenses/Roboto-NOTICE.txt`.
- Cousine-Regular: SIL Open Font License 1.1, see `licenses/OFL-1.1-Cousine.txt`.
