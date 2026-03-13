# Utilities for editing simple YAML-style settings files as plain text.
#
# readfile(filename)
#   Returns all file lines as Vector{String}.
#
# writefile(lines, filename)
#   Writes a Vector{String} back to disk, one line per entry.
#
# change_value(lines, varname, value)
#   Finds lines whose left-trimmed content starts with varname and replaces
#   the value field before a trailing '#' comment, preserving spacing and
#   comments. Numeric values are converted to text via repr(value).

# read as text
function readfile(filename)
    open(filename) do file
        readlines(file)
    end
end

function writefile(lines, filename)
    open(filename, "w") do file
        for line in lines
            write(file, line, '\n')
        end
    end
end

function change_value(lines, varname, value::Union{Integer, Float64})
    change_value(lines, varname, repr(value))
end

function change_value(lines, varname, value::String)
    res = String[]
    for line in lines
        if startswith(lstrip(line), varname)
            start = (findfirst(varname, line)).stop+1
            comment_pos = findfirst('#', line)
            stop = isnothing(comment_pos) ? lastindex(line) : comment_pos - 1
            new_line = ""
            leading = true
            j = 1
            for (i, chr) in pairs(line)
                if i < start || i > stop
                    new_line *= chr
                elseif line[i] == ' ' && leading
                    new_line *= ' '
                elseif j <= length(value)
                    new_line *= value[j]
                    j += 1
                    leading = false
                elseif i <= stop
                    new_line *= ' '
                end
            end
            push!(res, new_line)
        else
            push!(res, line)
        end
    end
    res
end
