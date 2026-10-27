# road_not_taken.jl
# Prints "The Road Not Taken" by Robert Frost.
#
# Setup: paste the poem text into a file named road_not_taken.txt
# in the same folder as this script, then run:
#     julia road_not_taken.jl

const POEM_FILE = joinpath(@__DIR__, "road_not_taken.txt")

const SUMMARY = """
"The Road Not Taken" by Robert Frost (1916)
A traveler stands where a path splits in two in a autumn wood.
He can't take both, so he picks one, telling himself he'll come back
to the other, though he knows he probably won't. Years later, he
imagines saying he chose the road less traveled, and that the choice
made all the difference.
"""

function main()
    if isfile(POEM_FILE)
        println(read(POEM_FILE, String))
    else
        println("Couldn't find $(basename(POEM_FILE)). Paste the poem into it and run again.\n")
        println(SUMMARY)
    end
end

main()
