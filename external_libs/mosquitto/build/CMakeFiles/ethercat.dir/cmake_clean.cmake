file(REMOVE_RECURSE
  "libethercat.pdb"
  "libethercat.a"
)

# Per-language clean rules from dependency scanning.
foreach(lang)
  include(CMakeFiles/ethercat.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
