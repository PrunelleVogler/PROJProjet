using JuMP
using CPLEX
using Libdl

function solve_program()
    #JULIA_STACK_SIZE=100000000
    include("Data/processed/500_USA-road-d.BAY.gr")
    # code = read("Data/processed/500_USA-road-d.BAY.gr", String)
    # e = Meta.parse(code)
    # eval(e)
    #nb_cities, s, t, S, d1, d2, p, ph, Mat, d, D = 4, 1, 4, 5, 10, 10, [1,1,1,1], [1,1,1,1], [[1,2], [2,3], [3,4]], [4,4,5], [1,1,1]
    nb_cities = n
    nb_edges = size(Mat)[1]
    
    m = Model(CPLEX.Optimizer)
    @variable(m, x[1:nb_edges], Bin)
    @variable(m, alpha >= 0)
    @variable(m, gamma >= 0)
    @variable(m, lambda[1:nb_edges] >= 0)
    @variable(m, beta[1:nb_cities] >= 0)

    @constraint(m, [e in 1:nb_edges], alpha + lambda[e] >= x[e])
    @constraint(m, 2 * sum(beta[i] for i in 1:nb_cities) + d2 * gamma <= S - p[s] - p[t] - sum(p[i] * x[e] for i in 1:nb_cities if i !=s for e in 1:nb_edges if Mat[e][1] == i ))
    for i in 1:nb_cities
        if i !=s && i !=t
            @constraint(m, [i in 1:nb_cities], gamma + beta[i] >= ph[i] * sum(x[e] for e in 1:nb_edges if convert(Int,Mat[e, 1]) == i))
        end
    end
    @constraint(m, gamma + beta[s] >= ph[s])
    @constraint(m, gamma + beta[t] >= ph[t])
    @constraint(m, sum(x[e] for e in 1:nb_edges if convert(Int,Mat[e, 1]) == s) == 1)
    @constraint(m, sum(x[e] for e in 1:nb_edges if convert(Int,Mat[e, 2]) == t) == 1)
    @constraint(m, sum(x[e] for e in 1:nb_edges if convert(Int,Mat[e, 2]) == s) == 0)
    @constraint(m, sum(x[e] for e in 1:nb_edges if convert(Int,Mat[e, 1]) == t) == 0)
    for i in 1:nb_cities
        if i !=s && i !=t
            @constraint(m, sum(x[e] for e in 1:nb_edges if convert(Int,Mat[e, 1]) == i) == sum(x[e] for e in 1:nb_edges if convert(Int,Mat[e, 2]) == i))
        end
    end
    
    @objective(m, Min, d1 * alpha + sum(Mat[e, 3] * x[e] + Mat[e, 4] * lambda[e] for e in 1:nb_edges))
    optimize!(m)

    feasibleSolutionFound = primal_status(m) == MOI.FEASIBLE_POINT
    isOptimal = termination_status(m) == MOI.OPTIMAL

    if feasibleSolutionFound
        println("Solution trouvée de ", s, " a ", t)
        vX = JuMP.value.(x)
        println("Valeurs de x")
        for e in 1:nb_edges
            if vX[e] == 1
                println(Mat[e, 1], " ", Mat[e, 2], " ")
            end
        end
        #println("Obj ", isOptimal.value)
    end
end

solve_program()