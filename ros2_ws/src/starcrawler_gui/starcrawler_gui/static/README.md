# three.js local para la vista 3D

La vista 3D (`/3d`) usa three.js. Si en esta carpeta hay un
`three.module.min.js`, `gui_node` lo sirve él mismo; si no, la página lo pide
al CDN de jsDelivr.

Eso significa que **sin la copia local, la vista 3D necesita internet en el
equipo que la abre**. En el laboratorio da igual; en el campo, con el portátil
conectado solo al robot, no carga.

Para dejar la copia, desde la raíz del repo y con internet:

```bash
curl -L -o ros2_ws/src/starcrawler_gui/starcrawler_gui/static/three.module.min.js https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.min.js
```

Y volver a compilar el paquete. La versión tiene que ser la misma que
`THREE_CDN` en `vista3d.py` (0.160.0): la página está escrita contra esa.
