# SPDX-FileCopyrightText: 2025 Uwe Fechner
# SPDX-License-Identifier: MIT

# Build and display the html documentation locally.

using Pkg

if !("LiveServer" ∈ keys(Pkg.project().dependencies))
    Pkg.activate("docs")
end
using LiveServer; servedocs()
