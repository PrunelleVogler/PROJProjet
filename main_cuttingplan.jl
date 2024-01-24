using JuMP
using CPLEX
using Libdl


function cutting_plane_algorithm()
    include("Data/processed/60_USA-road-d.BAY.gr")
    nb_cities = n
    nb_edges = size(Mat)[1]
    
    #================Master problem==================#
    m = Model(CPLEX.Optimizer)
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

    # Contraintes U1 et U2
    @constraint(m, z >= sum( x[e] * Mat[e, 3] for e in 1:nb_edges))
    @constraint(m, sum( y[i] * p[i] for i in 1:nb_cities) <= S)

    @objective(m, Min, z)

    #============Separation problem 1==================#
    m1 = Model(CPLEX.Optimizer)
    @variable(m1, 0 <= delta1[1:nb_edges])
    @variable(m1, a, Bin)
    @constraint(m1, sum(delta1[e] for e in 1:nb_edges) <= d1)
    @constraint(m1, [e in 1:nb_edges], delta1[e] <= Mat[e, 4])
    @objective(m1, Max, 1)

    #============Separation problem 2==================#
    m2 = Model(CPLEX.Optimizer)
    @variable(m2, delta2[1:nb_cities] >=0)
    @variable(m2, b, Bin)
    @constraint(m2, sum(delta2[i] for i in 1:nb_cities) <= d2)
    @constraint(m2, [i in 1:nb_cities], delta2[i] <= 2)
    @objective(m2, Max, 1)
    
    no_optimal_solution_found = true
    iteration = 0
    while no_optimal_solution_found && iteration < 10
        optimize!(m)
        feasibleSolutionFound = primal_status(m) == MOI.FEASIBLE_POINT
        
        if feasibleSolutionFound
            x_val = JuMP.value.(x)
            z_val = JuMP.value.(z)
            y_val = JuMP.value.(y)
            println("Solution master pb trouvee")

            # Modifier le pb 1 et le résoudre
            new_objective1 = @expression(m1, sum((delta1[e] + Mat[e, 3]) * x_val[e] for e in 1:nb_edges))
            set_objective_function(m1, new_objective1)
            optimize!(m1)

            # Modifier le pb 2 et le résoudre
            new_objective2 = @expression(m2, p[s] + p[t] + delta2[s] * ph[s] + delta2[t] * ph[t] + sum((delta2[i] * ph[i] + p[i]) * y_val[i] for i in 1:nb_cities if i !=s for e in 1:nb_edges if Mat[e][1] == i ))
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

            if z_val >= obj1 && obj2 <= S
                no_optimal_solution_found = false
                println("Solution optimale trouvée en ", iteration, " iterations")
            end
            if z_val < obj1
                delta1_val = JuMP.value.(delta1)
                @constraint(m, z >= sum((delta1_val[e] + 1) * Mat[e, 3] * x[e] for e in 1:nb_edges))
                println("Ajout coupe 1")
            end
            if obj2 > S
                delta2_val = JuMP.value.(delta2)
                @constraint(m, p[s] + p[t] + delta2_val[s] * ph[s] + delta2_val[t] * ph[t] + sum((delta2_val[i] * ph[i] + p[i]) * y[e] for i in 1:nb_cities if i !=s for e in 1:nb_edges if Mat[e][1] == i ) <= S)
                println("Ajout coupe 2")
            end
        else
            println("Problème de résolution du master problem")
        end
        iteration += 1
        #no_optimal_solution_found = false
    end
    z_val = JuMP.value.(z)
    println("Valeur objectif : ", z_val)
    # feasibleSolutionFound = primal_status(m) == MOI.FEASIBLE_POINT
    # isOptimal = termination_status(m) == MOI.OPTIMAL

    # if feasibleSolutionFound
    #     println("Solution trouvée de ", s, " a ", t)
    #     vX = JuMP.value.(x)
    #     println("Valeurs de x")
    #     for e in 1:nb_edges
    #         if vX[e] == 1
    #             println(Mat[e, 1], " ", Mat[e, 2], " ")
    #         end
    #     end
    #     #println("Obj ", isOptimal.value)
    # end
end

cutting_plane_algorithm()