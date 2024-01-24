using JuMP
using CPLEX
using Libdl

time_limit = 100

function isIntegerPoint(cb_data::CPLEX.CallbackContext, context_id::Clong)
    # cas 1 - une solution entière a été obtenue; ou
    # cas 2 - une relaxation non bornée a été obtenue
    if context_id != CPX_CALLBACKCONTEXT_CANDIDATE
        return false
    end
    # Pour déterminer si on est dans le cas 1 ou 2, on essaie de récupérer la
    # solution entière courante
    ispoint_p = Ref{Cint}()
    ret = CPXcallbackcandidateispoint(cb_data, ispoint_p)
    # S’il n’y a pas de solution entière
    if ret != 0 || ispoint_p[] == 0
        return false
    else
        return true
    end
end




include("Data/processed/20_USA-road-d.BAY.gr")
nb_cities = n
nb_edges = size(Mat)[1]

#================Master problem==================#
m = Model(CPLEX.Optimizer)
set_optimizer_attribute(m,"CPX_PARAM_TILIM",time_limit)
MOI.set(m, MOI.NumberOfThreads(), 1) #pour pouvoir utiliser les callbacks
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
set_optimizer_attribute(m1,"CPX_PARAM_SCRIND",0)

@variable(m1, 0 <= delta1[1:nb_edges])
@variable(m1, a, Bin)
@constraint(m1, sum(delta1[e] for e in 1:nb_edges) <= d1)
@constraint(m1, [e in 1:nb_edges], delta1[e] <= Mat[e, 4])
@objective(m1, Max, 1)

#============Separation problem 2==================#
m2 = Model(CPLEX.Optimizer)
set_optimizer_attribute(m2,"CPX_PARAM_SCRIND",0)

@variable(m2, delta2[1:nb_cities] >=0)
@variable(m2, b, Bin)
@constraint(m2, sum(delta2[i] for i in 1:nb_cities) <= d2)
@constraint(m2, [i in 1:nb_cities], delta2[i] <= 2)
@objective(m2, Max, 1)

# x_opt = zeros(nb_edges)

function callback(cb_data::CPLEX.CallbackContext, context_id::Clong)

    if isIntegerPoint(cb_data, context_id)

        CPLEX.load_callback_variable_primal(cb_data, context_id)

        z_val = callback_value(cb_data, z)
        x_val = callback_value.(Ref(cb_data), m[:x]) 
        y_val = callback_value.(Ref(cb_data), m[:y]) 


        # println("Callback value : z_val ", z_val)
        # println("Callback value : x_val ", x_val)
        # println("Callback value : y_val ", y_val)
        

        # Separation problem 1
        new_objective1 = @expression(m1, sum(Mat[e, 3]*(delta1[e] + 1) * x_val[e] for e in 1:nb_edges))
        set_objective_function(m1, new_objective1)
        optimize!(m1)
        obj1 = objective_value(m1)

        # Separation problem 2
        new_objective2 = @expression(m2, sum((delta2[i] * ph[i] + p[i]) * y_val[i] for i in 1:nb_cities))
        set_objective_function(m2, new_objective2)
        optimize!(m2)
        obj2 = objective_value(m2)
    

        # Callbacks et ajout de coupe
        # Coupe 1
        if  z_val  < obj1 
            delta1_val = JuMP.value.(delta1)
            cstr = @build_constraint(z >= sum((delta1_val[e] + 1) * Mat[e, 3] * x[e] for e in 1:nb_edges))
            MOI.submit(m, MOI.LazyConstraint(cb_data), cstr)
            # println("Ajout coupe 1")
        end       
        # Coupe 2
        if  obj2 > S
            delta2_val = JuMP.value.(delta2)
            cstr = @build_constraint(sum((delta2_val[i] * ph[i] + p[i]) * y[i] for i in 1:nb_cities) <= S)
            MOI.submit(m, MOI.LazyConstraint(cb_data), cstr)
            # println("Ajout coupe 2")
        end
    end
end

MOI.set(m, CPLEX.CallbackFunction(), callback)
optimize!(m)

# z_val = JuMP.value.(z)
# println("Valeur objectif : ", z_val)
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

# print(x_opt)
