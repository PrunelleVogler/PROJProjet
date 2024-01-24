using JuMP
using CPLEX
using Libdl


function cutting_plane_algorithm()
    include("Data/processed/500_USA-road-d.BAY.gr")
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
    @variable(m1, delta1[1:nb_edges] >=0)
    @constraint(m1, sum(delta1[e] for e in 1:nb_edges) <= d1)
    @constraint(m1, [e in :nb_edges], delta1[e] <= Mat[e, 4])

    #============Separation problem 2==================#
    m2 = Model(CPLEX.Optimizer)
    @variable(m2, delta2[1:nb_cities] >=0)
    @constraint(m2, sum(delta2[i] for i in 1:nb_cities) <= d2)
    @constraint(m2, [i in :nb_cities], delta2[i] <= 2)
    
    no_optimal_solution_found = true

    while no_optimal_solution_found
        optimize!(m)
        x_val = JuMP.value.(x)
        z_val = JuMP.value.(z)
        y_val = JuMP.value.(y)
        feasibleSolutionFound = primal_status(m) == MOI.FEASIBLE_POINT
        
        if feasibleSolutionFound
            x_val = JuMP.value.(x)
            z_val = JuMP.value.(z)
            y_val = JuMP.value.(y)

            # Modifier le pb 1 et le résoudre
            @objective(m1, Max, sum((delta1[e] + Mat[e, 4]) * x_val[e] for e in 1:nb_edges))
            optimize!(m1)

            # Modifier le pb 1 et le résoudre
            @objective(m2, Max, p[s] + p[t] + delta2[s] * ph[s] + delta2[t] * ph[t] + sum((delta1[i] * ph[i] + p[i]) * x_val[e] for i in 1:nb_cities if i !=s for e in 1:nb_edges if Mat[e][1] == i ))
            optimize!(m2)

            if z_val >= obj1 && obj2 <= S
                no_optimal_solution_found = false
                println("Solution optimale trouvée")
            end
            if z_val < obj1
                delta1_val = JuMP.value.(delta1)
                @constraint(m, z >= sum((delta1_val[e] + Mat[e, 4]) * x[e] for e in 1:nb_edges))
            end
            if obj2 > S
                delta2_val = JuMP.value.(delta2)
                @constraint(m, p[s] + p[t] + delta2_val[s] * ph[s] + delta2_val[t] * ph[t] + sum((delta2_val[i] * ph[i] + p[i]) * x[e] for i in 1:nb_cities if i !=s for e in 1:nb_edges if Mat[e][1] == i ) <= S)
            end
        else
            println("Problème de résolution du master problem")
        end
    end

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