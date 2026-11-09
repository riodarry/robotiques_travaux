import asyncio
from bleak import BleakClient, BleakScanner, BleakError

# UUID et envoi texte du client graphique du professeur.
CHARACTERISTIC_NOTIFY_UUID = "0000ffe2-0000-1000-8000-00805f9b34fb"
CHARACTERISTIC_WRITE_UUID = "0000ffe3-0000-1000-8000-00805f9b34fb"
IDENTITE_TEST = "BT9 : moteurs arretes"
TAILLE_ENVOI = 20


def construireMessage(texte):
    commande = texte.strip()
    if not commande:
        return b""
    if len(commande) > 31 or any(ord(c) < 32 or ord(c) > 126 for c in commande):
        raise ValueError("Maximum 31 caracteres ASCII, sans retour a la ligne.")
    return commande.encode("ascii") + b"\n"


async def envoyerCommande(client, caracteristique, texte):
    message = construireMessage(texte)
    if not message:
        return
    proprietes = caracteristique.properties
    avecReponse = "write" in proprietes
    if not avecReponse and "write-without-response" not in proprietes:
        raise ValueError("Le module ne permet pas l'envoi sur cette caracteristique.")
    for debut in range(0, len(message), TAILLE_ENVOI):
        bloc = message[debut:debut + TAILLE_ENVOI]
        await client.write_gatt_char(caracteristique, bloc, response=avecReponse)
    print("Envoye :", texte.strip())


async def main():
    print("Partie 9 : verifier les commandes, moteurs arretes.")
    print("Recherche des robots Makeblock pendant 8 secondes...")
    appareils = await BleakScanner.discover(timeout=8)
    robots = [d for d in appareils if d.name and d.name.casefold().startswith("makeblock")]
    if not robots:
        print("Aucun robot trouve. Verifie son alimentation et son module Bluetooth.")
        return
    for numero, robot in enumerate(robots, start=1):
        print(f"{numero}. {robot.name} ({robot.address})")
    choix = await asyncio.to_thread(input, "Numero de TON robot, ou Entree pour annuler : ")
    if not choix.strip():
        return
    try:
        numero = int(choix)
    except ValueError:
        print("Numero invalide.")
        return
    if numero < 1 or numero > len(robots):
        print("Numero invalide.")
        return
    robot = robots[numero - 1]
    async with BleakClient(robot, timeout=15) as client:
        ecriture = client.services.get_characteristic(CHARACTERISTIC_WRITE_UUID)
        notifications = client.services.get_characteristic(CHARACTERISTIC_NOTIFY_UUID)
        if ecriture is None or notifications is None:
            print("Les caracteristiques du module Makeblock sont introuvables.")
            return
        identiteRecue = asyncio.Event()
        tampon = ""

        def recevoir(_caracteristique, donnees):
            nonlocal tampon
            tampon += donnees.decode("utf-8", errors="replace")
            while "\n" in tampon:
                ligne, tampon = tampon.split("\n", 1)
                ligne = ligne.rstrip("\r")
                if ligne == IDENTITE_TEST:
                    identiteRecue.set()
                else:
                    print(ligne, flush=True)
            if len(tampon) > 1024:
                tampon = ""

        await client.start_notify(notifications, recevoir)
        try:
            await asyncio.wait_for(identiteRecue.wait(), timeout=5)
        except asyncio.TimeoutError:
            print("Croquis partie 9 non confirme. Aucune commande envoyee.")
            print("Envoie d'abord partie_09_bluetooth dans le robot.")
            return
        print("Test 9 confirme. Commandes : cal, ligne, lab3, stop ; quit pour quitter.")
        while client.is_connected:
            texte = await asyncio.to_thread(input, "Commande : ")
            if texte.strip() == "quit":
                break
            if not client.is_connected:
                print("Connexion perdue. Relance le script pour te reconnecter.")
                break
            try:
                await envoyerCommande(client, ecriture, texte)
            except ValueError as erreur:
                print(erreur)
    print("Deconnecte.")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except (BleakError, OSError) as erreur:
        print("Erreur Bluetooth :", erreur)
    except (EOFError, KeyboardInterrupt):
        print("Fin du test.")
