using JuMP
using CPLEX
using Libdl
using TimerOutputs
epsilon = 0.0001
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

function cutting_plane_algorithm(instanceName, timeLimit)
    time_begin = time()
    n, s, t, S, d1, d2, p, ph, Mat = lecture(instanceName)
    nb_cities = n
    nb_edges = size(Mat)[1]
    
    #================Master problem==================#
    m = Model(CPLEX.Optimizer)
    set_optimizer_attribute(m, "CPX_PARAM_SCRIND", 0)
    @variable(m, x[1:nb_edges], Bin)
    @variable(m, z >= 0)
    @variable(m, y[1:nb_cities], Bin)
    
    # Contraintes de flot
    @constraint(m, sum(x[e] for e in 1:nb_edges if convert(Int,Mat[e, 1]) == s) == 1)
    @constraint(m, sum(x[e] for e in 1:nb_edges if convert(Int,Mat[e, 2]) == t) == 1)
    @constraint(m, sum(x[e] for e in 1:nb_edges if convert(Int,Mat[e, 2]) == s) == 0)
    @constraint(m, sum(x[e] for e in 1:nb_edges if convert(Int,Mat[e, 1]) == t) == 0)
    for i in 1:nb_cities
        if i !=s && i !=t
            @constraint(m, sum(x[e] for e in 1:nb_edges if convert(Int,Mat[e, 1]) == i) == sum(x[e] for e in 1:nb_edges if convert(Int,Mat[e, 2]) == i))
        end
    end
    #d_ij = Mat[ , 3]
    #D_ij = Mat[ , 4]
    # Contraintes U1 et U2
    @constraint(m, z >= sum( x[e] * Mat[e, 3] for e in 1:nb_edges))
    @constraint(m, sum( y[i] * p[i] for i in 1:nb_cities) <= S)

    @objective(m, Min, z)

    #============Separation problem 1==================#
    m1 = Model(CPLEX.Optimizer)
    set_optimizer_attribute(m1, "CPX_PARAM_SCRIND", 0)
    @variable(m1, 0 <= delta1[1:nb_edges])
    @variable(m1, a, Bin)
    @constraint(m1, sum(delta1[e] for e in 1:nb_edges) <= d1)
    @constraint(m1, [e in 1:nb_edges], delta1[e] <= Mat[e, 4])
    @objective(m1, Max, 1)

    #============Separation problem 2==================#
    m2 = Model(CPLEX.Optimizer)
    set_optimizer_attribute(m2, "CPX_PARAM_SCRIND", 0)
    @variable(m2, delta2[1:nb_cities] >=0)
    @variable(m2, b, Bin)
    @constraint(m2, sum(delta2[i] for i in 1:nb_cities) <= d2)
    @constraint(m2, [i in 1:nb_cities], delta2[i] <= 2)
    @objective(m2, Max, 1)
    
    no_optimal_solution_found = true
    iteration = 0
    upper_bound = 0
    lower_bound = 0
    x_val = zeros(nb_edges)
    while no_optimal_solution_found && (time() - time_begin < timeLimit)
        optimize!(m)
        feasibleSolutionFound = primal_status(m) == MOI.FEASIBLE_POINT
        
        if feasibleSolutionFound
            x_val = JuMP.value.(x)
            z_val = JuMP.value.(z)
            y_val = JuMP.value.(y)
            if iteration % 10 == 0
                println("Valeur master pb a l'iteration ", iteration, " : ", z_val)
            end

            # Modifier le pb 1 et le résoudre
            new_objective1 = @expression(m1, sum((delta1[e] + 1) * Mat[e, 3] * x_val[e] for e in 1:nb_edges))
            set_objective_function(m1, new_objective1)
            optimize!(m1)

            # Modifier le pb 2 et le résoudre
            new_objective2 = @expression(m2, sum((delta2[i] * ph[i] + p[i]) * y_val[i] for i in 1:nb_cities))
            set_objective_function(m2, new_objective2)
            optimize!(m2)
            
            feasibleSolutionFound1 = primal_status(m1) == MOI.FEASIBLE_POINT
            feasibleSolutionFound2 = primal_status(m2) == MOI.FEASIBLE_POINT
            if !feasibleSolutionFound1 || !feasibleSolutionFound2
                println("Pb de sép non résolu")
                no_optimal_solution_found = false
                break
            end
            obj1 = objective_value(m1)
            obj2 = objective_value(m2)

            if z_val >= obj1 - epsilon && obj2 <= S + epsilon
                no_optimal_solution_found = false
                println("Solution optimale trouvée en ", iteration, " iterations")
                upper_bound = objective_value(m)
                lower_bound = objective_bound(m)
            end
            if z_val < obj1
                delta1_val = JuMP.value.(delta1)
                @constraint(m, z >= sum((delta1_val[e] + 1) * Mat[e, 3] * x[e] for e in 1:nb_edges))
                #println("Ajout coupe 1")
            end
            if obj2 > S
                delta2_val = JuMP.value.(delta2)
                @constraint(m, sum((delta2_val[i] * ph[i] + p[i]) * y[i] for i in 1:nb_cities) <= S)
                #println("Ajout coupe 2")
            end
        else
            println("Problème de résolution du master problem")
        end
        iteration += 1
        #no_optimal_solution_found = false
    end
    if time() - time_begin >= timeLimit
        optimize!(m)
        if primal_status(m) == MOI.FEASIBLE_POINT
            upper_bound = 0
            lower_bound = objective_value(m)
        end
    end

    # feasibleSolutionFound = primal_status(m) == MOI.FEASIBLE_POINT
    # if feasibleSolutionFound
    #     println("Solution trouvée de ", s, " a ", t)
    #     vX = JuMP.value.(x)
    #     println("Valeurs de x")
    #     for e in 1:nb_edges
    #         if vX[e] == 1
    #             println(Mat[e, 1], " ", Mat[e, 2], " ")
    #         end
    #     end
    # end
    println("Upper bound ", upper_bound)
    println("Lower bound ", lower_bound)
    time_end = time()
    println("Time ", time_end - time_begin)
    return upper_bound, lower_bound, time_end - time_begin
end

cutting_plane_algorithm(instanceName, 2)