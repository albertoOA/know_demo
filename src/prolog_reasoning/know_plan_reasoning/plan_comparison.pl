:- module(plan_comparison,
    [ compare_all_existing_plans_in_pairs/0,
	  compare_two_plans(r,r), 
      compare_two_plans_based_on_cost(r,r),
	  compare_two_plans_based_on_number_of_tasks(r,r),
	  compare_two_plans_based_on_makespan(r,r), 
	  compare_two_plans_based_on_overall_quality(r,r)
    ]).

/** <module> Predicates for comparative reasoning with plans

@author Alberto Olivares-Alarcos
@license BSD
*/

%% compare_all_existing_plans_in_pairs() 
% 
% Compares all instances of a plan in pairs (e.g., based on their qualities and their roles) to decide which one is better. 
% The logic of this predicate is application dependent. 
%
%
compare_all_existing_plans_in_pairs() :-
	findall((Ia, Ib), (instance_of(Ia, dul:'Plan'), instance_of(Ib, dul:'Plan'), dif(Ia, Ib)), Pairs), 
	forall(member((A, B), Pairs), ignore(compare_two_plans(A,B))).

%% compare_two_plans(?Pa, ?Pb) 
% 
% Compares two plans (e.g., based on their qualities and their roles) to decide which one is better. 
% The logic of this predicate is application dependent. 
%
% @param Pa         Instance of dul:'Plan'
% @param Pb         Instance of dul:'Plan'
%
compare_two_plans(Pa, Pb) :-
	compare_two_plans_based_on_cost(Pa, Pb),
	compare_two_plans_based_on_number_of_tasks(Pa, Pb),
	compare_two_plans_based_on_makespan(Pa, Pb), 
	compare_two_plans_based_on_overall_quality(Pa, Pb). 


%% compare_two_plans_based_on_cost(?Pa, ?Pb) 
% 
% Compares the cost of two plans (e.g., based on their values).
%
% @param Pa         Instance of dul:'Plan'
% @param Pb         Instance of dul:'Plan'
%
compare_two_plans_based_on_cost(Pa, Pb) :-
	triple(Pa, ocra_common:'hasCost', Xa), 
	triple(Pb, ocra_common:'hasCost', Xb), 
	triple(Xa, dul:'hasDataValue', Va), 
	triple(Xb, dul:'hasDataValue', Vb), 
	(ground(Xa), ground(Xb)) ->
	((Va == Vb) -> kb_project(triple(Xa, ocra_common:'hasEquivalentQualityValueThan', Xb)), kb_project(triple(Xb, ocra_common:'hasEquivalentQualityValueThan', Xa)), kb_project(triple(Pb, ocra_common:'isPlanWithSameCostAs', Pa)), kb_project(triple(Pa, ocra_common:'isPlanWithSameCostAs', Pb)) ;	
	((Va < Vb) -> kb_project(triple(Xa, ocra_common:'hasBetterQualityValueThan', Xb)), kb_project(triple(Xb, ocra_common:'hasWorseQualityValueThan', Xa)), kb_project(triple(Pb, ocra_common:'isMoreExpensivePlanThan', Pa)), kb_project(triple(Pa, ocra_common:'isCheaperPlanThan', Pb)) ; 
	kb_project(triple(Xb, ocra_common:'hasBetterQualityValueThan', Xa)), kb_project(triple(Xa, ocra_common:'hasWorseQualityValueThan', Xb)), kb_project(triple(Pa, ocra_common:'isMoreExpensivePlanThan', Pb)), kb_project(triple(Pb, ocra_common:'isCheaperPlanThan', Pa)))) ; false. 


%% compare_two_plans_based_on_number_of_tasks(?Pa, ?Pb) 
% 
% Compares the number of tasks of two plans (e.g., based on their values).
%
% @param Pa         Instance of dul:'Plan'
% @param Pb         Instance of dul:'Plan'
%
compare_two_plans_based_on_number_of_tasks(Pa, Pb) :-
	triple(Pa, ocra_common:'hasNumberOfTasks', Xa), 
	triple(Pb, ocra_common:'hasNumberOfTasks', Xb), 
	triple(Xa, dul:'hasDataValue', Va), 
	triple(Xb, dul:'hasDataValue', Vb), 
	(ground(Xa), ground(Xb)) ->
	((Va == Vb) -> kb_project(triple(Xa, ocra_common:'hasEquivalentQualityValueThan', Xb)), kb_project(triple(Xb, ocra_common:'hasEquivalentQualityValueThan', Xa)), kb_project(triple(Pb, ocra_common:'isPlanWithSameNumberOfTasksAs', Pa)), kb_project(triple(Pa, ocra_common:'isPlanWithSameNumberOfTasksAs', Pb)) ;
	((Va < Vb) -> kb_project(triple(Xa, ocra_common:'hasBetterQualityValueThan', Xb)), kb_project(triple(Xb, ocra_common:'hasWorseQualityValueThan', Xa)), kb_project(triple(Pb, ocra_common:'isLongerPlanThan', Pa)), kb_project(triple(Pa, ocra_common:'isShorterPlanThan', Pb)) ; 
	kb_project(triple(Xb, ocra_common:'hasBetterQualityValueThan', Xa)), kb_project(triple(Xa, ocra_common:'hasWorseQualityValueThan', Xb)), kb_project(triple(Pa, ocra_common:'isLongerPlanThan', Pb)), kb_project(triple(Pb, ocra_common:'isShorterPlanThan', Pa)))) ; false. 


%% compare_two_plans_based_on_makespan(?Pa, ?Pb) 
% 
% Compares the makespan of two plans (e.g., based on their values).
%
% @param Pa         Instance of dul:'Plan'
% @param Pb         Instance of dul:'Plan'
%
compare_two_plans_based_on_makespan(Pa, Pb) :-
	triple(Pa, ocra_common:'hasExpectedMakespan', Xa), 
	triple(Pb, ocra_common:'hasExpectedMakespan', Xb), 
	triple(Xa, dul:'hasDataValue', Va), 
	triple(Xb, dul:'hasDataValue', Vb), 
	(ground(Xa), ground(Xb)) ->
	((Va == Vb) -> kb_project(triple(Xa, ocra_common:'hasEquivalentQualityValueThan', Xb)), kb_project(triple(Xb, ocra_common:'hasEquivalentQualityValueThan', Xa)), kb_project(triple(Pb, ocra_common:'isPlanWithSameExpectedMakespanAs', Pa)), kb_project(triple(Pa, ocra_common:'isPlanWithSameExpectedMakespanAs', Pb)) ;
	((Va < Vb) -> kb_project(triple(Xa, ocra_common:'hasBetterQualityValueThan', Xb)), kb_project(triple(Xb, ocra_common:'hasWorseQualityValueThan', Xa)), kb_project(triple(Pb, ocra_common:'isSlowerPlanThan', Pa)), kb_project(triple(Pa, ocra_common:'isFasterPlanThan', Pb)) ; 
	kb_project(triple(Xb, ocra_common:'hasBetterQualityValueThan', Xa)), kb_project(triple(Xa, ocra_common:'hasWorseQualityValueThan', Xb)), kb_project(triple(Pa, ocra_common:'isSlowerPlanThan', Pb)), kb_project(triple(Pb, ocra_common:'isFasterPlanThan', Pa)))) ; false. 


%% compare_two_plans_based_on_overall_quality(?Pa, ?Pb) 
% 
% Compares all the qualities of two plans to decide which if one of them is better.
%
% @param Pa         Instance of dul:'Plan'
% @param Pb         Instance of dul:'Plan'
%
compare_two_plans_based_on_overall_quality(Pa, Pb) :-
	triple(Pa, ocra_common:'hasCost', Ca), 
	triple(Pb, ocra_common:'hasCost', Cb), 
	triple(Pa, ocra_common:'hasNumberOfTasks', Ta), 
	triple(Pb, ocra_common:'hasNumberOfTasks', Tb),
	triple(Pa, ocra_common:'hasExpectedMakespan', Ma), 
	triple(Pb, ocra_common:'hasExpectedMakespan', Mb),
	(ground(Ca), ground(Cb), ground(Ta), ground(Tb), ground(Ma), ground(Mb)) ->
	((triple(Ca, ocra_common:'hasBetterQualityValueThan', Cb), triple(Ta, ocra_common:'hasBetterQualityValueThan', Tb), triple(Ma, ocra_common:'hasBetterQualityValueThan', Mb)) ->
	kb_project(triple(Pa, ocra:'isBetterPlanThan', Pb)),  kb_project(triple(Pb, ocra:'isWorsePlanThan', Pa)) ; 
	((triple(Ca, ocra_common:'hasEquivalentQualityValueThan', Cb), triple(Ta, ocra_common:'hasEquivalentQualityValueThan', Tb), triple(Ma, ocra_common:'hasEquivalentQualityValueThan', Mb)) ->
	kb_project(triple(Pa, ocra:'isEquivalentPlanTo', Pb)),  kb_project(triple(Pb, ocra:'isEquivalentPlanTo', Pa))) ; 
	((triple(Ca, ocra_common:'hasWorseQualityValueThan', Cb), triple(Ta, ocra_common:'hasWorseQualityValueThan', Tb), triple(Ma, ocra_common:'hasWorseQualityValueThan', Mb)) ->
	kb_project(triple(Pb, ocra:'isBetterPlanThan', Pa)),  kb_project(triple(Pa, ocra:'isWorsePlanThan', Pb)))) ; false.

