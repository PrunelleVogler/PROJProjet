using JuMP
using CPLEX
using Libdl
using TimerOutputs
using Distributions
epsilon = 0.0001
instanceName = "Data/processed/2100_USA-road-d.BAY.gr"
timeLimit = 600

function lecture(file)
    if isfile(file)
        # L’ouvrir
        myFile = open(file)
        # Lire toutes les lignes d’un fichier
        data = readlines(myFile)
        # Pour chaque ligne du fichier
        i = 1
        n, s, t, S, d1, d2 = 0, 0, 0, 0, 0, 0
        p = zeros(10)
        ph = zeros(10)
        Mat = zeros(10, 4)
        for line in data
            # Afficher la ligne
            if i == 1
                tab = split(line, " ")
                n = parse(Int64, tab[3])
                p = zeros(n)
                ph = zeros(n)
                Mat = zeros(size(data)[1], 4)
            end
            if i == 2
                tab = split(line, " ")
                s = parse(Int64, tab[3])
            end
            if i == 3
                tab = split(line, " ")
                t = parse(Int64, tab[3])
            end
            if i == 4
                tab = split(line, " ")
                S = parse(Int64, tab[3])
            end
            if i == 5
                tab = split(line, " ")
                d1 = parse(Int64, tab[3])
            end
            if i == 6
                tab = split(line, " ")
                d2 = parse(Int64, tab[3])
            end
            if i == 7
                tab1 = split(line, "p = [")
                tab2 = split(tab1[2], "]")
                tab = split(tab2[1], ", ")
                j = 1
                for element in tab
                    p[j] = parse(Int64, element)
                    j += 1
                end
            end
            if i == 8
                tab1 = split(line, "ph = [")
                tab2 = split(tab1[2], "]")
                tab = split(tab2[1], ", ")
                j = 1
                for element in tab
                    ph[j] = parse(Int64, element)
                    j += 1
                end
            end
            if i > 9 && i < size(data)[1]
                tab1 = split(line, ";")
                tab = split(tab1[1], " ")
                Mat[i - 9, 1] = parse(Int64, tab[1])
                Mat[i - 9, 2] = parse(Int64, tab[2])
                Mat[i - 9, 3] = parse(Int64, tab[3])
                Mat[i - 9, 4] = parse(Float64, tab[4])
            end
            if i == size(data)[1]
                tab1 = split(line, "]")
                tab = split(tab1[1], " ")
                Mat[i - 9, 1] = parse(Int64, tab[1])
                Mat[i - 9, 2] = parse(Int64, tab[2])
                Mat[i - 9, 3] = parse(Int64, tab[3])
                Mat[i - 9, 4] = parse(Float64, tab[4])
            end
            i += 1
        end
        # Fermer le fichier
        close(myFile)
    end
    return n, s, t, S, d1, d2, p, ph, Mat
end

function heuristic_algorithm(instanceName, timeLimit)
    time_begin = time()
    n, s, t, S, d1, d2, p, ph, Mat = lecture(instanceName)
    println("Fin lecture : ", time() - time_begin)
    nb_cities = n
    nb_edges = size(Mat)[1]
    #Number of different robust value in the objective
    nb_U1 = 10
    #Number of robust constraint considered
    nb_U2 = 10
    
    # Static model
    m = Model(CPLEX.Optimizer)
    set_optimizer_attribute(m, "CPX_PARAM_SCRIND", 0)
    @variable(m, x[1:nb_edges], Bin)
    @variable(m, y[1:nb_cities], Bin)
    
    @constraint(m, sum(x[e] for e in 1:nb_edges if convert(Int,Mat[e, 1]) == s) == 1)
    @constraint(m, sum(x[e] for e in 1:nb_edges if convert(Int,Mat[e, 2]) == t) == 1)
    @constraint(m, sum(x[e] for e in 1:nb_edges if convert(Int,Mat[e, 2]) == s) == 0)
    @constraint(m, sum(x[e] for e in 1:nb_edges if convert(Int,Mat[e, 1]) == t) == 0)
    for i in 1:nb_cities
        if i !=s && i !=t
            @constraint(m, sum(x[e] for e in 1:nb_edges if convert(Int,Mat[e, 1]) == i) == sum(x[e] for e in 1:nb_edges if convert(Int,Mat[e, 2]) == i))
        end
    end

    # Stochastic robust constraints
    for i in 1:nb_U2
        delta = zeros(nb_cities)
        for k in 1:nb_cities
            delta[k] = rand(Uniform(0, 2))
        end
        somme = sum(delta[k] for k in 1:nb_cities)
        while somme > d2
            k = round(rand(1:nb_cities))
            delta[k] = 0
            somme = sum(delta[k] for k in 1:nb_cities)
        end
        @constraint(m, sum(y[i] * (p[i] + delta[i] * ph[i]) for i in 1:nb_cities) <= S)
    end
    @objective(m, Min, 0)
    println("Fin ajout contraintes robustes : ", time() - time_begin)

    # Solve nb_U1 problems with different objective
    x_val = zeros((nb_U1, nb_edges))
    d_1 = zeros((nb_U1, nb_edges))
    for k in 1:nb_U1
        for e in 1:nb_edges
            if 0 < Mat[e, 4]
                d_1[k,e] = rand(Uniform(0, Mat[e, 4]))
            end
        end
        somme = sum(d_1[k,e] for e in 1:nb_edges)
        while somme > d1
            e = rand(1:nb_edges)
            if d_1[k,e] > 0
                d_1[k,e] = rand(Uniform(0, d_1[k,e]))
            end
            somme = sum(d_1[k,e] for e in 1:nb_edges)
        end
        new_objective = @expression(m, sum(x[e] * Mat[e, 3] * (1 + d_1[k,e]) for e in 1:nb_edges))
        set_objective_function(m, new_objective)
        set_optimizer_attribute(m, "CPX_PARAM_TILIM", timeLimit / nb_U1)
        optimize!(m)
        
        x_val_prime = JuMP.value.(x)
        for e in 1:nb_edges
            x_val[k, e] = x_val_prime[e]
        end
    end
    println("Fin calcul des solutions : ", time() - time_begin)
    
    #Find the solution of min max
    Val_heuristic = 100000000
    for k in 1:nb_U1
        max_x_k = 0
        for j in 1:nb_U1
            d_1_x_k = sum(x_val[k, e] * Mat[e, 3] * (1 + d_1[j,e]) for e in 1:nb_edges)
            if d_1_x_k > max_x_k
                max_x_k = d_1_x_k
            end
        end
        if max_x_k < Val_heuristic
            Val_heuristic = max_x_k
        end
    end
    println("val : ", Val_heuristic)

    return Val_heuristic, time() - time_begin
end

#heuristic_algorithm(instanceName, 2)