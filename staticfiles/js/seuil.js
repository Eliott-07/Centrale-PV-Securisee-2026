document.addEventListener('DOMContentLoaded', function() {
    const sliderActivation = document.getElementById('seuil-activation-range');
    const sliderDesactivation = document.getElementById('seuil-desactivation-range');
    
    if (!sliderActivation || !sliderDesactivation) return;

    // Créer les conteneurs pour les affichages
    const containerActivation = document.createElement('div');
    containerActivation.className = 'slider-container';
    sliderActivation.parentNode.insertBefore(containerActivation, sliderActivation);
    containerActivation.appendChild(sliderActivation);

    const containerDesactivation = document.createElement('div');
    containerDesactivation.className = 'slider-container';
    sliderDesactivation.parentNode.insertBefore(containerDesactivation, sliderDesactivation);
    containerDesactivation.appendChild(sliderDesactivation);

    // Créer les éléments d'affichage de la température
    const tempDisplayActivation = document.createElement('div');
    tempDisplayActivation.className = 'temperature-display';
    containerActivation.appendChild(tempDisplayActivation);

    const tempDisplayDesactivation = document.createElement('div');
    tempDisplayDesactivation.className = 'temperature-display';
    containerDesactivation.appendChild(tempDisplayDesactivation);

    // Créer les affichages numériques à côté des sliders
    const valueDisplayActivation = document.createElement('div');
    valueDisplayActivation.className = 'value-display';
    containerActivation.appendChild(valueDisplayActivation);

    const valueDisplayDesactivation = document.createElement('div');
    valueDisplayDesactivation.className = 'value-display';
    containerDesactivation.appendChild(valueDisplayDesactivation);

    // Fonction de mise à jour de l'affichage
    function updateTempDisplay() {
        const valeurActivation = parseInt(sliderActivation.value);
        const valeurDesactivation = parseInt(sliderDesactivation.value);

        // Mise à jour des affichages principaux
        tempDisplayActivation.textContent = `Seuil d'activation : ${valeurActivation}°C`;
        tempDisplayDesactivation.textContent = `Seuil de désactivation : ${valeurDesactivation}°C`;

        // Mise à jour des affichages numériques
        valueDisplayActivation.textContent = `${valeurActivation}°C`;
        valueDisplayDesactivation.textContent = `${valeurDesactivation}°C`;

        // Validation visuelle
        if (valeurDesactivation >= valeurActivation) {
            tempDisplayDesactivation.style.color = 'red';
            tempDisplayDesactivation.innerHTML = `Seuil de désactivation : ${valeurDesactivation}°C <span class="warning">⚠️ Le seuil de désactivation doit être inférieur au seuil d'activation</span>`;
            valueDisplayDesactivation.style.color = '#ff4444';
        } else {
            tempDisplayDesactivation.style.color = '#333';
            tempDisplayDesactivation.innerHTML = `Seuil de désactivation : ${valeurDesactivation}°C`;
            valueDisplayDesactivation.style.color = '#333';
        }

        // Mise à jour des couleurs des sliders
        sliderActivation.style.setProperty('--slider-color', '#4CAF50');
        sliderDesactivation.style.setProperty('--slider-color', valeurDesactivation >= valeurActivation ? '#ff4444' : '#4CAF50');
    }

    // Initialiser l'affichage
    updateTempDisplay();

    // Mettre à jour l'affichage lors du déplacement des sliders
    sliderActivation.addEventListener('input', updateTempDisplay);
    sliderDesactivation.addEventListener('input', updateTempDisplay);
}); 