time_limit = 360

### Lecture de l'ensemble des instances
# nb_villes = [20]

# # nb_villes = [20,40,60,80,100,120,140,160,180,200,250,300,350,400,450,500,550,600,650,700,750,800,850,900,950,1000,1100,1200,1300,1400,1500,1600,1700,1800,1900,2000,2100,2200,2300,2400,2500]
# area = ["BAY", "COL", "NY"]

# # Initialiser une liste pour stocker les termes
# list_instances = []

# # Créer la liste des termes
# for nb in nb_villes
#     for a in area
#         push!(list_instances, "$(nb)_USA-road-d.$a.gr")
#     end
# end
# list_instances = [
#     "20_USA-road-d.NY.gr",
#     "60_USA-road-d.COL.gr",
#     "100_USA-road-d.BAY.gr",
#     "140_USA-road-d.NY.gr",
#     "180_USA-road-d.COL.gr",
#     "250_USA-road-d.NY.gr",
#     "350_USA-road-d.COL.gr",
#     "450_USA-road-d.NY.gr",
#     "550_USA-road-d.COL.gr",
#     "650_USA-road-d.NY.gr",
#     "750_USA-road-d.BAY.gr",
#     "850_USA-road-d.COL.gr",
#     "950_USA-road-d.NY.gr",
#     "1100_USA-road-d.NY.gr",
#     "1300_USA-road-d.NY.gr",
#     "1500_USA-road-d.NY.gr",
#     "1700_USA-road-d.NY.gr",
#     "1900_USA-road-d.BAY.gr",
#     "2100_USA-road-d.COL.gr",
#     "2300_USA-road-d.BAY.gr",
#     "2500_USA-road-d.NY.gr"
# ]
list_instances = [
    "20_USA-road-d.NY.gr",
    "60_USA-road-d.COL.gr",
    "100_USA-road-d.BAY.gr",
    "140_USA-road-d.NY.gr",
    "180_USA-road-d.COL.gr",
    "250_USA-road-d.NY.gr",
    "350_USA-road-d.COL.gr",
    "450_USA-road-d.NY.gr",
    "550_USA-road-d.COL.gr"
]
### Ouvrir fichier txt


### Executer dual/cutting plane/ branch and cutting et remplir le fichier txt
include("main_cuttingplan.jl")
# file_name = "cutting_plane"*"-"*string(minimum(nb_villes))*"-"*string(maximum(nb_villes))*"-"*string(time_limit)*"s.txt"
# file_name = "cutting_plane-comparaison-méthodes"*string(time_limit)*"s.txt"

# include("main_dual.jl")
# file_name = "dual"*"-"*string(minimum(nb_villes))*"-"*string(maximum(nb_villes))*"-"*string(time_limit)*"s.txt"

# include("branch_and_cut.jl")
# file_name = "branch_and_cut"*"-"*string(minimum(nb_villes))*"-"*string(maximum(nb_villes))*"-"*string(time_limit)*"s.txt"
file = open(file_name,"w")

for instance in list_instances

    # Execution dual
    println("Data/processed/"*instance)
    instance_name = "Data/processed/"*instance 
    # upper_bound, lower_bound, execution_time = dual_algorithm(instance_name, time_limit)
    upper_bound, lower_bound, execution_time = cutting_plane_algorithm(instance_name, time_limit)
    # upper_bound, lower_bound, execution_time, path = dual_algorithm(instance_name, time_limit)
    # write(file, join((instance, string(execution_time), string(lower_bound), string(upper_bound), string(abs(upper_bound - lower_bound) / upper_bound),path), ";"))
    write(file, join((instance, string(execution_time), string(lower_bound), string(upper_bound), string(abs(upper_bound - lower_bound) / upper_bound)), ";"))
    write(file, "\n")
end

close(file)



