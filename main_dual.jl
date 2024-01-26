using JuMP
using CPLEX
using Libdl
instanceName = "Data/processed/20_USA-road-d.BAY.gr"

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

function dual_algorithm(instanceName, timeLimit)
    time_begin = time()
    n, s, t, S, d1, d2, p, ph, Mat = lecture(instanceName)
    nb_cities = n
    nb_edges = size(Mat)[1]
    
    m = Model(CPLEX.Optimizer)
    @variable(m, x[1:nb_edges], Bin)
    @variable(m, y[1:nb_cities], Bin)
    @variable(m, alpha >= 0)
    @variable(m, gamma >= 0)
    @variable(m, lambda[1:nb_edges] >= 0)
    @variable(m, beta[1:nb_cities] >= 0)

    @constraint(m, [e in 1:nb_edges], alpha + lambda[e] >= x[e] * Mat[e, 3])
    @constraint(m, sum(2 * beta[i] + p[i] * y[i] for i in 1:nb_cities) + d2 * gamma <= S )

    @constraint(m, [i in 1:nb_cities], gamma + beta[i] >= ph[i] * y[i])

    @constraint(m, sum(x[e] for e in 1:nb_edges if convert(Int,Mat[e, 1]) == s) == 1)
    @constraint(m, sum(x[e] for e in 1:nb_edges if convert(Int,Mat[e, 2]) == t) == 1)
    @constraint(m, sum(x[e] for e in 1:nb_edges if convert(Int,Mat[e, 2]) == s) == 0)
    @constraint(m, sum(x[e] for e in 1:nb_edges if convert(Int,Mat[e, 1]) == t) == 0)
    for i in 1:nb_cities
        if i != t
            @constraint(m, sum(x[e] for e in 1:nb_edges if convert(Int,Mat[e, 1]) == i) == y[i])
        end
        if i != s
            @constraint(m, sum(x[e] for e in 1:nb_edges if convert(Int,Mat[e, 2]) == i) == y[i])
        end
    end

    
    for i in 1:nb_cities
        if i !=t && i !=s
            @constraint(m, sum(x[e] for e in 1:nb_edges if convert(Int,Mat[e, 1]) == i) == sum(x[e] for e in 1:nb_edges if convert(Int,Mat[e, 2]) == i))
        end
    end
    
    @objective(m, Min, d1 * alpha + sum(Mat[e, 3] * x[e] + Mat[e, 4] * lambda[e] for e in 1:nb_edges))
    set_optimizer_attribute(m, "CPX_PARAM_TILIM", timeLimit - (time() - time_begin))
    optimize!(m)

    # feasibleSolutionFound = primal_status(m) == MOI.FEASIBLE_POINT
    # if feasibleSolutionFound
    #     println("Solution trouvée de ", s, " a ", t)
    #     x_val = JuMP.value.(x)
    #     alpha_val = JuMP.value.(alpha)
    #     lambda_val = JuMP.value.(lambda)
    #     println("Valeurs de x")
    #     obj = d1 * alpha_val + sum(Mat[e, 3] * x_val[e] + Mat[e, 4] * lambda_val[e] for e in 1:nb_edges)
    #     for e in 1:nb_edges
    #         if x_val[e] == 1
    #             println(Mat[e, 1], " ", Mat[e, 2], " ")
    #         end
    #     end
    #     println("Obj ", obj)
    # end
    time_end = time()
    lower_bound = 0
    upper_bound = 0
    if primal_status(m) == MOI.OPTIMAL
        upper_bound = objective_value(m)
        lower_bound = objective_bound(m)
    end
    if primal_status(m) == MOI.FEASIBLE_POINT
        upper_bound = objective_value(m)
        lower_bound = objective_bound(m)
    end
    println("Upper bound ", upper_bound)
    println("Lower bound ", lower_bound)
    println("Time ", time_end - time_begin)
    return upper_bound, lower_bound, time_end - time_begin
end

dual_algorithm(instanceName, 10)