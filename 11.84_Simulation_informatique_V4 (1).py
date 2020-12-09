# Projet Q1
# Kazmi Simran, Kenfack Querol-Junior, Le Clercq Simon, Martens Matthieu, Wyns Lea

from math import *
import matplotlib.pyplot as plt
import numpy as np


### Constantes

g = 9.80665     # gravitation [m/s**2]
I = 0.1477      # moment d'inertie [kg*m**2]
D = 0.6         # coefficient d'amortissement [kg*s/m]

### Paramètres de la simulation

step = 0.0001   # time step    
end = 5.0      # time end 
theta_0 = 0     # angle initial      
ω_0 = 0.0       # vitesse initiale

t = np.arange(0, end, step)
theta = np.empty_like(t)
ω = np.empty_like(t)
ac = np.empty_like(t)
Ek=np.empty_like(t)
Eg=np.empty_like(t)
Eim=np.empty_like(t)
Ea=np.empty_like(t)
E_totale=np.empty_like(t)

# Fonctions

def define_all_variables():
    """
    pré: valeurs introduites par l'utilisateur avec la commande "init"
    post: enregistre ces valeurs sous forme de float
    """
    # On déclare que les variables sont globales afin de les utiliser dans les fonctions qui suivent
    global masse_barge
    global masse_grue
    global masse_deplacee
    global masse_restante
    global longueur_barge
    global largeur_barge
    global hauteur_barge
    global hauteur_grue
    global hauteur_masses_restantes
    global distance_deplacement
    # On demande à l'utilisateur d'initialiser les valeurs.
    # On utilise un bloc Try...Except qui nous permet de gérer les erreurs.
    try:
        masse_barge = float(input("Masse de la barge [kg] : "))
        masse_grue = float(input("Masse de la grue [kg] : "))
        masse_deplacee = float(input("Masse deplacee [kg] : "))
        masse_restante = float(input("Masse restante [kg] : "))
        longueur_barge = float(input("Longueur de la barge [m] : "))
        largeur_barge = float(input("Largeur de la barge [m] : "))
        hauteur_barge = float(input("Hauteur de la barge [m] : "))
        hauteur_grue = float(input("Hauteur du centre de gravité de la grue [m] : "))
        hauteur_masses_restantes = float(input("Hauteur des masses restantes [m] : "))
        distance_deplacement = float(input("Distance de deplacement [m] : "))
    except ValueError:
        print("Valeur invalide")

def print_values():
    """
    pré: commande "settings"
    post: affiche les valeurs actuellement enregistrées
    """
    # Après avoir rentré la commande "settings", la fonction est lancée.
    # Dans le bloc try, on retourne les valeurs initialisées par l'utilisateur.
    try:
        return("Masse de la barge = " + str(masse_barge) + " kg" + "\n"
               "Masse de la grue = " + str(masse_grue) + " kg" + "\n"
               "Masse déplacée = " + str(masse_deplacee) + " kg" + "\n"
               "Masse restante = " + str(masse_restante) + " kg" + "\n"
               "Longueur de la barge = " + str(longueur_barge) + " m" + "\n"
               "Largeur de la barge = " + str(largeur_barge) + " m" + "\n"
               "Hauteur de la barge = " + str(hauteur_barge) + " m" + "\n"
               "Hauteur du centre de gravité de la grue = " + str(hauteur_grue) + " m" + "\n"
               "Hauteur des masses restantes = " + str(hauteur_masses_restantes) + " m" + "\n"
               "Distance de déplacement : " + str(distance_deplacement) + " m")
    except NameError:
        print("Il manque une (ou plusieurs) valeur(s)")

def do_masse_totale(masse_barge,masse_grue,masse_deplacee,masse_restante):
    """
    pré: toutes les masses en kg
    post: masse totale en kg
    """
    global masse_totale
    masse_totale = masse_barge + masse_grue + masse_deplacee + masse_restante   # Calcule de la somme des différentes masses qui composent le système.
    return masse_totale
    
def do_volume_eau_deplace(masse_totale):
    """
    pré: masse totale en kg
    post: volume d'eau en m3
    """
    global volume_eau_deplace
    volume_eau_deplace = (masse_totale/1000)   # Calcule le volume d'eau déplacé par le système.
    return volume_eau_deplace
    
def do_ligne_flottaison(longueur_barge,largeur_barge,masse_totale):
    """
    pré: dimensions en m et masse totale en kg.
    post: hauteur de la partie immergée en m.
    """
    global ligne_flottaison
    ligne_flottaison = ((volume_eau_deplace)/(longueur_barge*largeur_barge))   # Calcule la hauteur de la ligne de flottaison, autrement dit, la hauteur de la partie immergée.
    return ligne_flottaison
    
def do_flottabilite(hauteur_barge,ligne_flottaison):
    """
    pré: la hauteur de la barge et la hauteur de la ligne de flottaison en m.
    post: Affiche si la barge coule.
    """
    if hauteur_barge >= ligne_flottaison :   # Si la ligne de flottaison est plus haute que la hauteur de la barge, la barge coule.
        return ("\033[32m" + "Oui" + "\033[30m")
    else :
        return ("\033[31m" + "LA BARGE COULE" + "\033[30m")
        
def do_hauteur_centre_gravité_total(hauteur_barge,masse_barge,hauteur_grue,masse_grue,masse_restante,hauteur_masses_restantes,masse_totale,ligne_flottaison,masse_deplacee):
    """
    pré: les dimensions en m et les masses des différents éléments du système en kg.
    post: la hauteur au dessus de la ligne de flottaison du centre de gravité du système en m.
    """
    # Calculs des centres de gravité individuels des différentes composantes du système.
    a = ((hauteur_barge * masse_barge)/2)
    b = ((hauteur_barge + (hauteur_grue/2)) * masse_grue)
    c = ((hauteur_barge + (hauteur_masses_restantes/2)) * (masse_restante + masse_deplacee)) 
    
    # Calcul du centre de gravité total du système.
    centre_gravite_axe_z = ((a + b + c) / masse_totale)
    global hauteur_centre_gravité_total
    hauteur_centre_gravité_total = centre_gravite_axe_z - ligne_flottaison
    return hauteur_centre_gravité_total
    
def do_inclinaison(ligne_flottaison,hauteur_barge,largeur_barge,longueur_barge,masse_totale,masse_barge,masse_deplacee,distance_deplacement,hauteur_centre_gravité_total):
    """
    Calcule l'angle d'inclinaison de la barge. Puis, retourne l'angle associé à la différence |C_r - C_a| la plus proche de 0.
    """
    theta = 0.0001
    liste_theta = []
    liste_diff = []
    global angle_max1
    angle_max1 = atan(((hauteur_barge - ligne_flottaison) * 2 ) / largeur_barge)
    global angle_max2
    angle_max2 = atan((ligne_flottaison * 2 ) / largeur_barge)
    while theta < (pi/2):
        # La partie de la barge immergée est en fait un trapèze composé d'un triangle et d'un rectangle.
        H_t = (largeur_barge / cos(theta))   # Hypothénuse du triangle.
        h_t = (largeur_barge * tan(theta))   # Hauteur du triangle (celle dans le prolongement de la largeur du rectangle).
        h_r = ((((2*(masse_totale/1000)) / (largeur_barge * longueur_barge)) - h_t) / 2)   # Largeur du rectangle.
        OG_r = ((h_t + h_r) / 2)   # Distance entre l'origine, milieu de l'hypothénuse du triangle, et le centre de gravité du rectangle.
        rho = (masse_barge / (longueur_barge * largeur_barge * hauteur_barge))   # Masse volumique de la barge           
        volume_r = (longueur_barge * largeur_barge * h_r)   # Volume du rectangle
        volume_t = ((longueur_barge * largeur_barge * h_t) / 2)   # Volume du triangle
        m_t = rho * volume_t   # Masse du triangle
        m_r = rho * volume_r   # Masse du rectangle
        
        # Nouvelles coordonnées du centre de gravité total.
        x_centre_gravite_bis = hauteur_centre_gravité_total * sin(theta)
        z_centre_gravite_bis = hauteur_centre_gravité_total * cos(theta)
        # Coordonnées du centre de gravité du rectangle.
        x_r = OG_r * (-sin(theta))
        z_r = OG_r * (-cos(theta))
        # Coordonnées du milieu du côté du triangle ayant comme longueur la largeur de la barge.
        x_a = h_t * ((-sin(theta)) / 2)
        z_a = h_t * ((-cos(theta)) / 2)
        # Coordonnées du sommet du triangle à l'intersection de l'hypothénuse et de la hauteur du triangle.
        x_b = H_t / 2
        z_b = 0
        # Coordonnées du centre de gravité du triangle.
        x_t = (((2 * x_a) + x_b) / 3)
        z_t = (((2 * z_a) + z_b) / 3)
        # Coordonnées du centre de gravité de la partie immergée.
        x_imm = ((((m_t) / (m_t + m_r)) * (x_t - x_r)) + x_r)
        z_imm = ((((m_t) / (m_t + m_r)) * (z_t - z_r)) + z_r)
        
        distance_orthogonale = (x_imm - x_centre_gravite_bis)   # Différence entre la coordonnée en x du centre de poussée et la coordonnée en x du centre de gravité totale.
        diff = distance_orthogonale - ((masse_deplacee * distance_deplacement) / masse_totale)
        diff_abs = abs(diff)
        liste_diff.append(diff_abs)
        liste_theta.append(theta)
        theta += 0.0001
    diff_min = liste_diff.index(min(liste_diff))
    global inclinaison
    inclinaison = liste_theta[diff_min]
    return inclinaison

def verification_inclinaison(angle_max1,angle_max2,inclinaison):
    """
    pré: les angles limites et l'inclinaison en radian.
    post: Affiche si la barge flotte ou non.
    """
    # Plus tôt, nous avons calculé les angles maximaux de soulèvement et de submersion.
    # La fonction affiche si la barge flotte, se soulève ou est submergée en fonction de l'inclinaison 
    if inclinaison < angle_max1 and inclinaison < angle_max2 :
        return("\033[32m" + "La barge flotte" + "\033[30m")
    if inclinaison >= angle_max2 :
        return("\033[31m" + "La barge se soulève" + "\033[30m")
    if inclinaison >= angle_max1 :
        return("\033[31m" + "La barge est submergée" + "\033[30m")
    
def optimisation_angle_max(longueur_barge,largeur_barge,hauteur_barge,masse_totale):
    """
    pré: les dimensions de la barge en m et la masse totale en kg
    post: la masse supplémentaire en kg à ajouter pour égaliser l'angle de soulèvement et de submersion en radian.
    """
    global m_supp
    m_supp = ((longueur_barge*largeur_barge*hauteur_barge)*500)-masse_totale
    return m_supp

def calcul_simulation(theta):
    """
    Calcule la distance entre la nouvelle position des coordonnées en x du centre de gravité et la nouvelle position des coordonnées en x du centre de poussée ainsi que le masse volumique de la barge et la hauteur du centre de poussée par rapport à la ligne de flottaison.
    """
    # Morceau du code de do_inclinaison() permettant de donner différentes données importantes pour la simulation.
    H_t = (longueur_barge / cos(theta))
    h_t = (longueur_barge * tan(theta))
    h_r = ((((2*(masse_totale/1000)) / (largeur_barge * longueur_barge)) - h_t) / 2)
    OG_r = ((h_t + h_r) / 2)
    rho = (masse_barge / (longueur_barge * largeur_barge * hauteur_barge))
    volume_r = (longueur_barge * largeur_barge * h_r)
    volume_t = ((longueur_barge * largeur_barge * h_t) / 2)
    m_t = rho * volume_t
    m_r = rho * volume_r
    x_centre_gravite_bis = hauteur_centre_gravité_total * sin(theta)
    x_r = OG_r * (-sin(theta))
    z_r = OG_r * (-cos(theta))
    x_a = h_t * ((-sin(theta)) / 2)
    z_a = h_t * ((-cos(theta)) / 2)
    x_b = H_t / 2
    z_b = 0
    x_t = (((2 * x_a) + x_b) / 3)
    z_t = (((2 * z_a) + z_b) / 3)
    x_imm = ((((m_t) / (m_t + m_r)) * (x_t - x_r)) + x_r)
    z_imm = ((((m_t) / (m_t + m_r)) * (z_t - z_r)) + z_r)
    distance_orthogonale = x_imm - x_centre_gravite_bis
    return (distance_orthogonale, rho, z_imm)

def simulation():
    """
    pré: Constantes et paramètres de la simulation définis au début du programme
    post: Exécute une simulation jusqu'à t=end par pas de dt=step. Remplit les listes x, v, a des positions, vitesses et accélérations à l'aide de calcul de la somme algébrique des couples.
    """
    theta[0] = theta_0
    ω[0] = ω_0
    for i in range(len(t)-1):
        dt = step
        distance_orthogonale = calcul_simulation(theta[i])[0]
        rho = calcul_simulation(theta[i])[1]
        z_imm = calcul_simulation(theta[i])[2]
        
        # bilan des forces
        c_a = masse_deplacee * g * distance_deplacement
        c_r = masse_totale * g * distance_orthogonale
        c_total = c_a - c_r
        
        # calculs accélération, vitesse, angle
        ac[i] = (-D * ω[i] + c_total) / I 
        ω[i+1] = ω[i] + (ac[i] * dt)
        theta[i+1] = theta[i] + (ω[i] * dt)
        
        # calculs des énergies
        Ek[i+1] = (I * ((ω[i+1])**2))/2
        Eg[i+1] = masse_totale * g * ((cos(theta[i+1]) * hauteur_centre_gravité_total) - hauteur_centre_gravité_total)
        Ea[i+1] = -c_a * theta[i+1]
        Eim[i+1] = -(longueur_barge * largeur_barge * ligne_flottaison * rho) * g * (z_imm - (-ligne_flottaison/2))
        E_totale[i+1] = Ek[i+1] + Eg[i+1] + Eim[i+1] + Ea[i+1]
        
def graphique1():
    """
    Graphique représentant l'angle d'inclinaison du système en fonction du temps calculé dans la fonction simulation. L'angle limite de submersion y est représenté en rouge et l'angle limite de soulèvement en jaune.
    """
    plt.figure(1)
    plt.axhline(y=angle_max1, xmin=0, xmax=100, linestyle="dashed", color = "red", label="submersion")
    plt.axhline(y=angle_max2, xmin=0, xmax=100, linestyle="dashed", color = "gold", label="soulèvement")
    plt.axhline(y=-angle_max1, xmin=0, xmax=100, linestyle="dashed", color = "red")
    plt.axhline(y=-angle_max2, xmin=0, xmax=100, linestyle="dashed", color = "gold")
    plt.plot(t,theta, label="inclinaison theta")
    plt.xlabel("Temps en secondes")
    plt.ylabel("Angle en radians")
    plt.legend()
    plt.show()

def graphique2():
    """
    Deux graphiques sont apparents. Le premier représente la vitesse angulaire du système en fonction du temps. Le second représente l'accélération centripète du système en fonction du temps. Données calculées dans la fonction simulation.
    """
    plt.figure(1)
    plt.subplot(3,1,1)
    plt.plot(t,ω, label="vitesse angulaire")
    plt.xlabel("Temps en secondes")
    plt.ylabel("rad/s")
    plt.legend()
    plt.subplot(3,1,3)
    plt.plot(t,ac, label="accélération centripète")
    plt.xlabel("Temps en secondes")
    plt.ylabel("rad/sec^2")
    plt.legend()
    plt.show()

def graphique3():
    """
    Ce graphique représente les différentes énergies calculées dans la fonction simulation.
    """
    plt.figure(3)
    plt.subplot(1,1,1)
    plt.plot(t,Ek,label="EnergieCinétique")
    plt.plot(t,Eg,label="Energie gravitationnelle",color="orange")
    plt.plot(t,Eim,label="Energie immersion",color="green")
    plt.plot(t,Ea,label="Energie charge",color="black")
    plt.plot(t,E_totale,label="Energie totale",color="red")
    plt.xlabel("Temps en secondes")
    plt.ylabel("Joules")
    plt.legend()
    plt.show()
    
def graphique4():
    """
    Diagramme de phase représentant la vitesse angulaire en fonction de l'angle d'inclinaison.
    """
    plt.figure(4)
    plt.subplot(1,1,1)
    plt.plot(theta,ω,label="Diagramme de phase",color="purple")
    plt.xlabel("Angle en radians")
    plt.ylabel("Vitesse angulaire")
    plt.legend()
    plt.show()
    
def execute_all():
    '''
    Exécute toutes les fonctions à l'exception de celles liées aux graphiques.
    '''
    return("Masse totale : " + str(do_masse_totale(masse_barge,masse_grue,masse_deplacee,masse_restante)) + " kg" + "\n"
           "Volume d'eau déplacé : " + str(do_volume_eau_deplace(masse_totale)) + " m3" + "\n"
           "Ligne de flottaison : " + str(do_ligne_flottaison(longueur_barge,largeur_barge,masse_totale)) + " m" + "\n"
           "La barge flotte (sans inclinaison) ? " + do_flottabilite(hauteur_barge,ligne_flottaison) + "\n"
           "Centre de gravité total : " + str(do_hauteur_centre_gravité_total(hauteur_barge,masse_barge,hauteur_grue,hauteur_grue,masse_restante,hauteur_masses_restantes,masse_totale,ligne_flottaison,masse_deplacee)) + " m" + "\n"
           "Inclinaison de la barge : " + str(do_inclinaison(ligne_flottaison,hauteur_barge,largeur_barge,longueur_barge,masse_totale,masse_barge,masse_deplacee,distance_deplacement,hauteur_centre_gravité_total)) + " rad" + "\n"
           + verification_inclinaison(angle_max1,angle_max2,inclinaison) + "\n"
           "Masse supplementaire pour optimiser l'angle d'inclinaison max : " + str(optimisation_angle_max(longueur_barge,largeur_barge,hauteur_barge,masse_totale)) + " kg" + "/n")
      
# Main body

if __name__== "__main__":
    print("Bienvenue sur le calculateur de flottaison. Appuyez sur > enter < pour skip : ")
    while True :
        # Commandes demandées dans un bloc Try...Except, pour gérer les erreurs d'input.
        try :
            command_asked = input("\033[34m" + "Enter your command : " + "\033[30m")
            if command_asked == "help" :
                # Explication des différentes commandes supportées par ce programme.
                print(" init => initie les valeurs de départ \n settings => affiche les valeurs actuelles \n change setting => modifie la valeur précisée \n run => calcule toutes les valeurs \n exit => ferme le programme \n save => sauvegarde les paramètres/résultats sur un document \n > possibilité d'excecuter un seul calcul grâce à sa fonction <")
            
            elif command_asked == "init" :
                define_all_variables()
            
            elif command_asked == "settings" :
                print(print_values())
            
            elif command_asked == "change setting" :
                sub_command_asked = input("Quel paramètre voulez-vous modifier ? ")
                if sub_command_asked == "masse barge" :
                    masse_barge = float(input("Nouvelle masse de la barge : "))
            
                if sub_command_asked == "masse grue" :
                    masse_grue = float(input("Nouvelle masse de la grue : "))
                  
                if sub_command_asked == "masse deplacee" :
                    masse_deplacee = float(input("Nouvelle masse déplacée : "))
                
                if sub_command_asked == "masse restante" :
                    masse_restante = float(input("Nouvelle masse restante : "))
                    
                if sub_command_asked == "longueur barge" :
                    longueur_barge = float(input("Nouvelle longueur de la barge : "))
                
                if sub_command_asked == "largeur barge" :
                    largeur_barge = float(input("Nouvelle largeur de la barge : "))
                    
                if sub_command_asked == "hauteur barge" :    
                    hauteur_barge = float(input("Nouvelle hauteur de la barge : "))
                    
                if sub_command_asked == "hauteur grue" :    
                    hauteur_grue = float(input("Nouvelle hauteur de la grue : "))
                    
                if sub_command_asked == "hauteur des masses restantes" :    
                    hauteur_masses_restantes = float(input("Nouvelle hauteur des masses restantes : "))
                    
                if sub_command_asked == "distance de deplacement" :    
                    distance_deplacement = float(input("Nouvelle distance de deplacement : "))
                
            elif command_asked == "run" :
                # Affiche ce que la fonction execute_all retourne.
                # Pour rappel, execute_all lance toutes les fonctions exceptées celles liées aux graphiques.
                print(execute_all())
                # Lance les fonctions liées aux graphiques.
                simulation()
                graphique1()
                graphique2()
                graphique3()
                graphique4()
            
            elif command_asked == "masse totale" :
                print("Masse totale : " + str(do_masse_totale(masse_barge,masse_grue,masse_deplacee,masse_restante)) + " kg")
                
            elif command_asked == "volume eau" :
                print("Volume d'eau déplacé : " + str(volume_eau_deplace) + " m3")
                
            elif command_asked == "angle max" :
                print("Angle max : " + str(angle_max))
        
            elif command_asked == "inclinaison" :
                print("Inclinaison de la barge : " + str(do_inclinaison(ligne_flottaison,hauteur_barge,largeur_barge,longueur_barge,masse_totale,masse_barge,masse_deplacee,distance_deplacement)) + " rad")
            
            elif command_asked == "exit" :
                print("Si vous quittez le programme, toutes les valeurs seront supprimées")
                confirmation = input("Voulez-vous quitter ? oui pour quitter, autre pour continuer : ")
                if confirmation == "oui" :
                    break
                
            elif command_asked == "save" :
                file_name = input("Sur quel fichier voulez-vous enregistrer ces résultats ? Vous pouvez également en créer un nouveau en écrivant un nom : ")
                try:
                    with open(file_name,"a") as file :
                        file.write("Données : " + "\n" + str(print_values()) + "\n" + "Résultats : " + "\n" + str(execute_all()) + "\n")
                        print("\033[32m" + "Les données ont bien été sauvegardées" + "\033[30m")
                except TypeError:
                    print("Impossible de sauvegarder ces données")
                except NameError:
                    print("Impossible de sauvegarder ces données")
                except FileNotFoundError:
                    print("Ce fichier est introuvable")
                    
            else :
                print("Commande invalide")
                
        except ZeroDivisionError:
            print("Une valeur vaut 0")
        except ValueError:
            print("Il manque des valeurs pour effectuer ce calcul")
        except NameError:
            print("Valeurs non définies")