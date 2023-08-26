
function(get_filename_from_url url _OUTPUT_NAME)
    string(FIND "${url}" "/" pos REVERSE)
    math(EXPR pos "${pos} + 1")
    string(SUBSTRING "${url}" ${pos} -1 result)
    set("${_OUTPUT_NAME}" ${result} PARENT_SCOPE)
endfunction()
