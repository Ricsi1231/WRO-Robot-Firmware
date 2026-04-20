idf_component_register(
    SRCS "src/AStarSolver.cpp"
         "src/Heuristic.cpp"
    INCLUDE_DIRS "include"
    REQUIRES log Interfaces
)
