# Script de connexion rapide au robot
# Ouvre 2 fenetres : une qui lance ROS2 sur le robot, une avec la console de commande.
# L'IP tapee est memorisee dans last_ip.txt pour ne pas avoir a la retaper a chaque fois.

$ErrorActionPreference = "Stop"
$ipFile = Join-Path $PSScriptRoot "last_ip.txt"

$defaultIp = ""
if (Test-Path $ipFile) {
    $defaultIp = (Get-Content $ipFile -Raw).Trim()
}

$prompt = "Adresse IP du robot"
if ($defaultIp -ne "") { $prompt = "$prompt [$defaultIp]" }
$ip = Read-Host $prompt

if ([string]::IsNullOrWhiteSpace($ip)) {
    if ($defaultIp -eq "") {
        Write-Host "Aucune IP connue pour le moment. Trouve l'IP du robot (Net Analyzer, nmap -sn, ou ecran+clavier branches sur le Pi) puis relance ce script." -ForegroundColor Red
        Read-Host "Appuie sur Entree pour fermer"
        exit 1
    }
    $ip = $defaultIp
} else {
    Set-Content -Path $ipFile -Value $ip
}

Write-Host ""
Write-Host "ATTENTION : les commandes qui suivent peuvent faire tourner les moteurs du robot." -ForegroundColor Yellow
$confirm = Read-Host "Le robot est-il sur une cale, roues degagees, et as-tu de la place autour ? (tape oui pour continuer)"
if ($confirm -ne "oui") {
    Write-Host "Annule. Mets le robot en securite avant de relancer ce script." -ForegroundColor Red
    Read-Host "Appuie sur Entree pour fermer"
    exit 1
}

$remoteLaunch = "cd Navigation/robot_ws && source install/setup.bash && ros2 launch robot robot_launch.py"
$remoteConsole = "cd Navigation/robot_ws && source install/setup.bash && ros2 run robot console"

Write-Host ""
Write-Host "Connexion a navigation@$ip ..." -ForegroundColor Cyan
Write-Host "(Si la cle SSH n'est pas configuree, le mot de passe 'navigation' sera demande dans chaque fenetre)" -ForegroundColor DarkGray

Start-Process powershell -ArgumentList "-NoExit", "-Command", "ssh -t navigation@$ip `"$remoteLaunch`""
Start-Sleep -Seconds 4
Start-Process powershell -ArgumentList "-NoExit", "-Command", "ssh -t navigation@$ip `"$remoteConsole`""

Write-Host "Deux fenetres ouvertes : lancement ROS2 (robot_launch.py) et la console de commande." -ForegroundColor Green
Write-Host "Tape 'help' dans la fenetre console pour voir les commandes disponibles." -ForegroundColor Green
