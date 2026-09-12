---
layout: page
title: Fotos e vídeos
---

<div class="gallery">
  <div class="photo">
    <img src="assets/img/Eggbots.jpg" alt="Foto 1">
    <p>Logo del equipo, porque en la pagina principal no quiere aparecer</p>
  </div>
  <div class="photo">
    <img src="assets/fotos/battlebot.jpg" alt="Foto 2">
    <p>Montaxe da placa controladora ESP32</p>
  </div>
  <div class="photo">
    <img src="assets/fotos/battlebot_3D.jpg" alt="Foto 3">
    <p>Prototipo de chasis impreso en 3D</p>
  </div>
</div>

<style>
.gallery {
  display: flex;
  flex-wrap: wrap;
  gap: 20px;
  justify-content: center;
}
.photo {
  flex: 1 1 200px;
  text-align: center;
}
.photo img {
  max-width: 100%;
  border-radius: 8px;
  box-shadow: 0 2px 6px rgba(0,0,0,0.3);
}
.photo p {
  margin-top: 0.5em;
  font-size: 0.9em;
}
</style>