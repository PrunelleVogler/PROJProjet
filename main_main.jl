# Définir les valeurs de xxx et yyy
nb_villes = [20,40,60,80,100,120,140,160,180,200,250,300,350,400,450,500,550,600,650,700,750,800,850,900,950,1000,1100,1200,1300,1400,1500,1600,1700,1800,1900,2000,2100,2200,2300,2400,2500]
area = ["BAY", "COL", "NY"]

# Initialiser une liste pour stocker les termes
liste_terms = []

# Créer la liste des termes
for nb in nb_villes
    for a in area
        push!(liste_terms, "$(nb)_USA-road-d.$a.gr")
    end
end

# Afficher la liste des termes
println(liste_terms)
