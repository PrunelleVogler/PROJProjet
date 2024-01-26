using CSV



### Lecture de l'ensemble des instance
nb_villes = [20,40,60,80,100,120,140,160,180,200,250,300,350,400,450,500,550,600,650,700,750,800,850,900,950,1000,1100,1200,1300,1400,1500,1600,1700,1800,1900,2000,2100,2200,2300,2400,2500]
area = ["BAY", "COL", "NY"]

# Initialiser une liste pour stocker les termes
list_instances = []

# Créer la liste des termes
for nb in nb_villes
    for a in area
        push!(list_instances, "$(nb)_USA-road-d.$a.gr")
    end
end


### Ouvrir CSV
# Lire le fichier CSV existant
csv_file = "PROJDonnéesDual.csv"
data = CSV.File(csv_file) |> DataFrame

# Fonction pour mettre à jour le fichier CSV
function update_csv(instance_name, execution_time, objective, bound, gap)
    # Trouver l'index de la ligne correspondant à l'instance actuelle
    instance_index = findfirst(data[:, "Instance"] .== instance_name)

    # Mettre à jour les colonnes appropriées
    data[instance_index, "Time"] = execution_time
    data[instance_index, "Obj"] = objective
    data[instance_index, "Borne"] = bound
    data[instance_index, "Gap"] = gap

    # Écrire les modifications dans le fichier CSV
    CSV.write(csv_file, data)
end



### Executer dual/cutting plane/ branch and cutting et remplir CSV

for instance in list_instances
    
    # Mettre à jour le fichier CSV
    update_csv(instance_name, execution_time, objective, bound, gap)
end
