import numpy as np
import skfuzzy as fuzz
import matplotlib.pyplot as plt
from skfuzzy import control as ctrl



def fuzzy_control (y_actual, y_desired, theta_actual, theta_desired, omega_actual, omega_desired ):

# Generate universe variables
#   * Quality and service on subjective ranges [0, 10]
#   * Tip has a range of [0, 25] in units of percentage points
    LateralDisplacement = ctrl.Antecedent(np.arange(-3, 3, 0.5), 'LateralDisplacement')
    SteeringAngle1  = ctrl.Consequent(np.arange(-0.4, 0.4, 0.01),'SteeringAngle1')


# Generate fuzzy membership functions
    LateralDisplacement['R'] = fuzz.trapmf(LateralDisplacement.universe, [-100, -3, -1.5, 0])
    LateralDisplacement['C'] = fuzz.trimf(LateralDisplacement.universe, [-1.5, 0, 2])
    LateralDisplacement['L'] = fuzz.trapmf(LateralDisplacement.universe,[0,2,3,100])

#LateralDisplacement.view()


    SteeringAngle1['R'] = fuzz.trimf(SteeringAngle1.universe, [-0.4, -0.4, 0])
    SteeringAngle1['C'] = fuzz.trimf(SteeringAngle1.universe, [-0.005, 0, 0.005])
    SteeringAngle1['L'] = fuzz.trimf(SteeringAngle1.universe,[0,0.4,0.4])


#SteeringAngle1.view()


    rule_1 = ctrl.Rule(LateralDisplacement['C'] , SteeringAngle1['C'])
    rule_2 = ctrl.Rule(LateralDisplacement['R'] , SteeringAngle1['R'])
    rule_3 = ctrl.Rule(LateralDisplacement['L'] , SteeringAngle1['L'])
    SteeringAngle1_cntrl = ctrl.ControlSystem([rule_1, rule_2, rule_3])
    SteeringAngle1_out = ctrl.ControlSystemSimulation(SteeringAngle1_cntrl)



    # >>>>>>>>>>>>>>>>>>>>>>>> goz2 wa7oshh
    YawAngleError = ctrl.Antecedent(np.arange(-0.5, 0.5, 0.01), 'AngleError')
    YawRateError = ctrl.Antecedent(np.arange(-1, 1, 0.01),'RateError')
    SteeringAngle  = ctrl.Consequent(np.arange(-0.5, 0.5, 0.01),'SteeringAngle')


    # Generate fuzzy membership functions
    YawAngleError['NB'] = fuzz.trapmf(YawAngleError.universe, [-1, -0.5, -0.4, -0.2])
    YawAngleError['NM'] = fuzz.trimf(YawAngleError.universe, [-0.35, -0.2, -0.05])
    YawAngleError['NS'] = fuzz.trimf(YawAngleError.universe,[-0.2,-0.1,0])
    YawAngleError['ZE'] = fuzz.trimf(YawAngleError.universe,[-0.05, 0, 0.05])
    YawAngleError['PS'] = fuzz.trimf(YawAngleError.universe,[0, 0.1, 0.2])
    YawAngleError['PM'] = fuzz.trimf(YawAngleError.universe,[0.05, 0.2, 0.35])
    YawAngleError['PB'] = fuzz.trapmf(YawAngleError.universe,[0.2, 0.4,0.5, 1])

    #YawAngleError.view()

    YawRateError['NB'] = fuzz.trapmf(YawRateError.universe, [-2, -1, -0.8, -0.2])
    YawRateError['NM'] = fuzz.trimf(YawRateError.universe, [-0.8, -0.4, 0])
    YawRateError['ZE'] = fuzz.trimf(YawRateError.universe,[-0.2, 0, 0.2])
    YawRateError['PM'] = fuzz.trimf(YawRateError.universe,[0, 0.4, 0.8])
    YawRateError['PB'] = fuzz.trapmf(YawRateError.universe,[0.2, 0.8, 1, 2])

    #YawRateError.view()

    SteeringAngle['NB'] = fuzz.trapmf(SteeringAngle.universe, [-1, -0.5, -0.4, -0.2])
    SteeringAngle['NM'] = fuzz.trimf(SteeringAngle.universe, [-0.35, -0.2, -0.05])
    SteeringAngle['NS'] = fuzz.trimf(SteeringAngle.universe,[-0.2,-0.1,0])
    SteeringAngle['ZE'] = fuzz.trimf(SteeringAngle.universe,[-0.05, 0, 0.05])
    SteeringAngle['PS'] = fuzz.trimf(SteeringAngle.universe,[0, 0.1, 0.2])
    SteeringAngle['PM'] = fuzz.trimf(SteeringAngle.universe,[0.05, 0.2, 0.35])
    SteeringAngle['PB'] = fuzz.trapmf(SteeringAngle.universe,[0.2, 0.4,0.5, 1])

    #SteeringAngle.view()


    rule1 = ctrl.Rule(YawAngleError['NB'] & YawRateError['NB'], SteeringAngle['NB'])
    rule2 = ctrl.Rule(YawAngleError['NB'] & YawRateError['NM'], SteeringAngle['NB'])
    rule3 = ctrl.Rule(YawAngleError['NB'] & YawRateError['ZE'], SteeringAngle['NM'])
    rule4 = ctrl.Rule(YawAngleError['NB'] & YawRateError['PM'], SteeringAngle['NM'])
    rule5 = ctrl.Rule(YawAngleError['NB'] & YawRateError['PB'], SteeringAngle['NS'])
    rule6 = ctrl.Rule(YawAngleError['NM'] & YawRateError['NB'], SteeringAngle['NB'])
    rule7 = ctrl.Rule(YawAngleError['NM'] & YawRateError['NM'], SteeringAngle['NM'])
    rule8 = ctrl.Rule(YawAngleError['NM'] & YawRateError['ZE'], SteeringAngle['NM'])
    rule9 = ctrl.Rule(YawAngleError['NM'] & YawRateError['PM'], SteeringAngle['NS'])
    rule10 = ctrl.Rule(YawAngleError['NM'] & YawRateError['PB'], SteeringAngle['NS'])
    rule11 = ctrl.Rule(YawAngleError['NS'] & YawRateError['NB'], SteeringAngle['NM'])
    rule12 = ctrl.Rule(YawAngleError['NS'] & YawRateError['NM'], SteeringAngle['NM'])
    rule13 = ctrl.Rule(YawAngleError['NS'] & YawRateError['ZE'], SteeringAngle['NM'])
    rule14 = ctrl.Rule(YawAngleError['NS'] & YawRateError['PM'], SteeringAngle['NS'])
    rule15 = ctrl.Rule(YawAngleError['NS'] & YawRateError['PB'], SteeringAngle['NS'])
    rule16 = ctrl.Rule(YawAngleError['ZE'] & YawRateError['NM'], SteeringAngle['ZE'])
    rule17 = ctrl.Rule(YawAngleError['ZE'] & YawRateError['ZE'], SteeringAngle['ZE'])
    rule18 = ctrl.Rule(YawAngleError['ZE'] & YawRateError['PB'], SteeringAngle['ZE'])
    rule19 = ctrl.Rule(YawAngleError['ZE'] & YawRateError['NB'], SteeringAngle['ZE'])
    rule20 = ctrl.Rule(YawAngleError['ZE'] & YawRateError['PM'], SteeringAngle['ZE'])
    rule21 = ctrl.Rule(YawAngleError['PS'] & YawRateError['NB'], SteeringAngle['PS'])
    rule22 = ctrl.Rule(YawAngleError['PS'] & YawRateError['NM'], SteeringAngle['PS'])
    rule23 = ctrl.Rule(YawAngleError['PS'] & YawRateError['ZE'], SteeringAngle['PM'])
    rule24 = ctrl.Rule(YawAngleError['PS'] & YawRateError['PM'], SteeringAngle['PM'])
    rule25 = ctrl.Rule(YawAngleError['PS'] & YawRateError['PB'], SteeringAngle['PM'])
    rule26 = ctrl.Rule(YawAngleError['PM'] & YawRateError['NB'], SteeringAngle['PS'])
    rule27 = ctrl.Rule(YawAngleError['PM'] & YawRateError['NM'], SteeringAngle['PS'])
    rule28 = ctrl.Rule(YawAngleError['PM'] & YawRateError['ZE'], SteeringAngle['PM'])
    rule29 = ctrl.Rule(YawAngleError['PM'] & YawRateError['PM'], SteeringAngle['PM'])
    rule30 = ctrl.Rule(YawAngleError['PM'] & YawRateError['PB'], SteeringAngle['PB'])
    rule31 = ctrl.Rule(YawAngleError['PB'] & YawRateError['NB'], SteeringAngle['PS'])
    rule32 = ctrl.Rule(YawAngleError['PB'] & YawRateError['NM'], SteeringAngle['PM'])
    rule33 = ctrl.Rule(YawAngleError['PB'] & YawRateError['ZE'], SteeringAngle['PM'])
    rule34 = ctrl.Rule(YawAngleError['PB'] & YawRateError['PM'], SteeringAngle['PB'])
    rule35 = ctrl.Rule(YawAngleError['PB'] & YawRateError['PB'], SteeringAngle['PB'])

    SteeringAngle2_cntrl = ctrl.ControlSystem([rule1, rule2, rule3,rule4,rule5,rule6,rule7,rule8,rule9,rule10,rule11,rule12,rule13,rule14,rule15,rule16,rule17,rule18,rule19,rule20,rule21,rule22,rule23,rule24,rule25,rule26,rule27,rule28,rule29,rule30,rule31,rule32,rule33,rule34,rule35])

    SteeringAngle2_out = ctrl.ControlSystemSimulation(SteeringAngle2_cntrl)


# FINAL STEP AFTER 2 FUZZY 
 
 
#>>>>> wahsh fuzzy
    thetaError = theta_actual - theta_desired
    OmegaError = omega_actual - omega_desired
    YError = y_actual - y_desired
    SteeringAngle2_out.input['AngleError'] = thetaError
    SteeringAngle2_out.input['RateError'] = OmegaError

    SteeringAngle2_out.compute()

    St_wahsh= SteeringAngle2_out.output['SteeringAngle']
#>>>>> gasser fuzzy 

    SteeringAngle1_out.input['LateralDisplacement'] = YError

    SteeringAngle1_out.compute()

    St_gasser= SteeringAngle1_out.output['SteeringAngle1']


#final result
    fuzzy_out= St_gasser* 0.6 + St_wahsh *0.4

    return fuzzy_out

