fichier_entree = 'my_test_data1.txt'
fichier_sortie = 'test_data_norm1.txt'

with open(fichier_entree, 'r') as f_entree, open(fichier_sortie, 'w') as f_sortie:
    for ligne in f_entree:
        try:
            valeur = float(ligne.strip())
            nouvelle_valeur = valeur / 10
            f_sortie.write(f"{nouvelle_valeur:.3f}\n")  # Format à 2 décimales
        except ValueError:
            print(f"Attention: la ligne '{ligne}' n'est pas un nombre valide et a été ignorée.")

print(f"Résultats écrits dans {fichier_sortie} avec 2 décimales.")