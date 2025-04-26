import os
import re

def extract_paths_from_cmake(cmake_file):
    """Estrae i percorsi che seguono ${CMAKE_SOURCE_DIR} dal file CMakeLists.txt."""
    paths = []
    pattern = re.compile(r'\$\{CMAKE_SOURCE_DIR\}(/[^\s)]+)')
    
    with open(cmake_file, 'r', encoding='utf-8') as file:
        for line in file:
            matches = pattern.findall(line)
            paths.extend(matches)
    
    return paths

def ensure_paths_exist(paths, base_dir):
    """Controlla se i percorsi esistono, se non esistono li crea."""
    for rel_path in paths:
        full_path = os.path.join(base_dir, rel_path.strip('/'))
        
        if not os.path.exists(full_path):
            if '.' in os.path.basename(full_path):  # Se è un file
                os.makedirs(os.path.dirname(full_path), exist_ok=True)
                with open(full_path, 'w', encoding='utf-8') as f:
                    if full_path.endswith('.h'):
                        header_guard = os.path.basename(full_path).replace('.', '_').upper()
                        f.write(f"#ifndef {header_guard}\n#define {header_guard}\n\n#endif\n")
                    print(f"Creato file: {full_path}")
            else:  # Se è una directory
                os.makedirs(full_path, exist_ok=True)
                print(f"Creata directory: {full_path}")
        else:
            print(f"Esiste già: {full_path}")

def main():
    cmake_file = "CMakeLists.txt"  # Modifica se necessario
    base_dir = os.getcwd()  # Percorso base del progetto
    
    paths = extract_paths_from_cmake(cmake_file)
    ensure_paths_exist(paths, base_dir)

if __name__ == "__main__":
    main()