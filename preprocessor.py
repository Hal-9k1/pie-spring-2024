import sys
import os

class ModuleInfo:
    __slots__ = "name", "func_call", "body_text"
    def __init__(self, name, func_call, body):
        self.name = name
        self.func_call = func_call
        self.body_text = body

def process_file(file_path, indent=" " * 4, module_name=None, module_list=None, import_cursor=0):
    """Preprocesses a python script by recursively transcluding imported files."""
    # if not top level, returns the script without import statements
    if not os.path.exists(file_path):
        #print(file_path)
        return
    if module_list == None:
        module_list = []
        is_top_level = True
        entry_point_line_nums = []
    else:
        is_top_level = False
        if any(module_info.name == module_name for module_info in module_list):
            raise RuntimeError(f"Detected cyclic import of module {module_name}.")
    with open(file_path, "r", encoding="utf-8") as file:
        module_buffer = []
        while True:
            line = file.readline()
            if not line:
                break
            words = line.strip().split(" ")
            if not len(words):
              module_buffer.append(line)  
            elif words[0] == "import" or words[0] == "from":
                path_segments = words[1].split(".")
                imported_module_name = path_segments[-1].strip()
                prev_imported_module = next((module for module in module_list if module.name == imported_module_name), None)
                module_exports = f"_HELPER_module_export_dict['{imported_module_name}']"
                if prev_imported_module:
                    func_call = prev_imported_module.func_call
                else:
                    imported_body_text = process_file(os.path.join(*path_segments).strip() + ".py",
                        indent=indent, module_name=imported_module_name, module_list=module_list,
                        import_cursor=import_cursor + 1)
                    if not imported_body_text:
                        # module not found. assume it's built in and leave the import statement intact
                        module_buffer.append(line)
                        continue
                    func_call = f"_HELPER_import_{imported_module_name}()"
                    imported_module_buffer = [
                        f"def {func_call}:",
                        f"{indent}if '{imported_module_name}' in _HELPER_module_export_dict:",
                        f"{indent * 2}return",
                        "",
                        f"{indent}# Begin imported file."
                    ]
                    imported_module_buffer.extend([indent + line for line in imported_body_text.splitlines()])
                    imported_module_buffer.append(
                        f"\n{indent}# End imported file.\n"
                        f"{indent}{module_exports} = locals()\n\n\n"
                    )
                if words[0] == "import" and (len(words) < 3 or words[2] != "as"):
                    import_mode = "import"
                    after_import_word_idx = 2
                elif words[0] == "import":
                    import_mode = "import as"
                    after_import_word_idx = 4
                elif words[0] == "from" and (len(words) < 5 or words[4] != "as"):
                    import_mode = "from import"
                    after_import_word_idx = 4
                elif words[0] == "from":
                    import_mode = "from import as"
                    after_import_word_idx = 6
                else:
                    raise RuntimeError("Typo?")
                #import_only_line = " ".join(line.strip().split(" ")[:after_import_word_idx])
                import_line = f"{func_call}; "
                if import_mode == "import":
                    import_line += f"{imported_module_name} = _HELPER_Module('{imported_module_name}')"
                elif import_mode == "import as":
                    import_line += f"{words[3]} = _HELPER_Module('{imported_module_name}')"
                elif import_mode == "from import":
                    if words[3] == "*":
                        import_line += f"for k, v in {module_exports}.items(): exec(k + \" = v\")"
                    else:
                        import_line += f"{words[3]} = {module_exports}[\"{words[3]}\"]"
                elif import_mode == "from import as":
                    import_line += f"{words[5]} = {module_exports}[\"{words[3]}\"]"
                else:
                    raise RuntimeError("Typo?")
                import_line += " # " + line.strip() + "\n"
                module_buffer.append(import_line)

                if not prev_imported_module:
                    module_list.insert(import_cursor, ModuleInfo(imported_module_name, func_call,
                        "\n".join(imported_module_buffer)))
            else:
                if words[0] == "def" and is_top_level:
                    module_buffer.append(line[:line.find("d")] + "@_HELPER_entry_point\n")
                    entry_point_line_nums.append(len(module_buffer))
                module_buffer.append(line)
        if is_top_level:
            strings = [
                f"_HELPER_module_export_dict = {{}}",
                f"_HELPER_entry_point_line_nums = [{', '.join(str(num) for num in entry_point_line_nums)}]",
                f"class _HELPER_Module:",
                f"{indent}def __init__(self, module_name):",
                f"{indent * 2}self.__dict__ = _HELPER_module_export_dict[module_name]",
                f"{indent}def __getitem__(self, key):",
                f"{indent * 2}return self.__dict__[key]",
                f"{indent}def __setitem__(self, key, value):",
                f"{indent * 2}self.__dict__[key] = value",
                f"def _HELPER_entry_point(func):",
                f"{indent}import functools",
                f"{indent}@functools.wraps(func)",
                f"{indent}def wrapped(*args, **kwargs):",
                f"{indent * 2}try:",
                f"{indent * 3}return func(*args, **kwargs)",
                f"{indent * 2}except Exception as e:",
                f"{indent * 3}print('Source traceback (most recent call last):')",
                f"{indent * 3}frame_lines = []",
                f"{indent * 3}tb = e.__traceback__",
                f"{indent * 3}while tb:",
                f"{indent * 4}translation_result = _HELPER_translate_line_no(tb.tb_lineno)",
                f"{indent * 4}if not translation_result:",
                f"{indent * 5}tb = tb.tb_next",
                f"{indent * 5}continue",
                f"{indent * 4}module_name, line_no = translation_result",
                f"{indent * 4}frame_lines.append(f'  File \"{{module_name + \".py\"}}\", line {{line_no}}, in {{tb.tb_frame.f_code.co_name}}')",
                f"{indent * 4}tb = tb.tb_next",
                f"{indent * 3}print('\\n'.join(frame_lines))",
                f"{indent * 3}print(type(e).__name__ + (': ' if str(e) else '') + str(e))",
                f"{indent * 3}exit()",
                f"{indent}return wrapped",
                f"def _HELPER_translate_line_no(line_no):",
            ]
            strings = [string + "\n" for string in strings]
            running_line_num = len(strings) + 8 + 2 * len(module_list) # two lines added per module
            module_line_entries = []
            for module in module_list:
                module_line_entries.append(
                    f"{indent}elif line_no >= {running_line_num}:\n"
                    f"{indent * 2}return '{module.name}', line_no - {running_line_num + 5}\n"
                )
                running_line_num += module.body_text.count("\n")
            module_line_entries.append(
                f"{indent}if line_no >= {running_line_num}:\n"
                f"{indent * 2}skipped_lines = 0\n"
                f"{indent * 2}for entry_point_line_num in _HELPER_entry_point_line_nums:\n"
                f"{indent * 3}if entry_point_line_num + {running_line_num} <= line_no:\n"
                f"{indent * 4}skipped_lines += 1\n"
                f"{indent * 3}else:\n"
                f"{indent * 4}break\n"
                f"{indent * 2}return '{''.join(os.path.basename(file_path).split('.')[:-1])}', line_no - {running_line_num} - skipped_lines\n"
            )
            strings.extend(reversed(module_line_entries))
            strings.extend(module.body_text for module in module_list)
            strings.append("# End imports.\n")
            strings.extend(module_buffer)
            return "".join(strings)
        else:
            return "".join(module_buffer)

if __name__ == "__main__":
    print(process_file(sys.argv[1]))
