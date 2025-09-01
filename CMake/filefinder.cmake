#查找所有目录和其子目录，包括只包含子目录不包含文件的目录，但不包括空目录
# @param root_dir 根目录
# @param output_var 输出变量名
# @Author: gsh
# @Date: 2025-5-25

function(find_recursive_include_dirs root_dir output_var)
    # 查找所有 .cpp 和 .h 文件
    file(GLOB_RECURSE SRC_FILES 
        ${root_dir}/*.cpp 
        ${root_dir}/*.h
        ${root_dir}/*.c
        ${root_dir}/*.s
    )

    set(include_dirs)

    foreach(file_path ${SRC_FILES})
        get_filename_component(dir ${file_path} DIRECTORY)
        file(RELATIVE_PATH rel_dir ${CMAKE_CURRENT_SOURCE_DIR} ${dir})

        set(path_acc "")
        string(REPLACE "/" ";" path_parts ${rel_dir}) # 将路径分割成部分
        foreach(part ${path_parts})
            if(path_acc)
                set(path_acc "${path_acc}/${part}")
            else()
                set(path_acc "${part}")
            endif()
            list(APPEND include_dirs "${CMAKE_CURRENT_SOURCE_DIR}/${path_acc}")
        endforeach()
    endforeach()

    # 去重后设置输出变量到父作用域,连接在已有变量之后
    list(REMOVE_DUPLICATES include_dirs)
    set(${output_var} "${${output_var}};${include_dirs}" PARENT_SCOPE)
endfunction()
