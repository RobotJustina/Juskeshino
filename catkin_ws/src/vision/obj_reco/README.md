# Nodo Obj_Reco

Remake del nodo que hizo el Sr. Jebús.
La idea de este nodo es hacer reconocimiento de objetos del modo jebusiano: segmentando objetos arriba de un plano para luego reconocerlos usando únicamente el histograma del canal H de los objetos segmentados pasados a HSV.

### Tópicos suscritos:

Ninguno

### Tópicos publicados:

* /vision/obj_reco/markers (visualization_msgs/Marker): Publica una línea cuando se pide el borde de una mesa mediante el servicio /vision/line_finder/find_table_edge. Se publica una línea que dura 10 segundos visible. También se publica una caja casi plana que representa el plano obtenido cuando se atiende el servicio  /vision/line_finder/find_horizontal_plane_ransac.
* /vision/obj_reco/marker_array (visualization_msgs/MarkerArray):  Se publica cuando se atiende cualquier servicio de reconocimiento de objetos. Se publica una caja con las dimensiones del objeto reconocido con el color principal del objeto reconocido con un alpha. Se visualiza por 10 segs y desaparece.
* /vision/obj_reco/resulting_cloud (sensor_msgs/PointCloud2): Nube de puntos que contiene solo los puntos que están arriba del plano horizontal más grande detectado. Se publica después de atender el servicio /vision/get_points_above_plane.

### Servicios requeridos:
Ninguno

### Servicios atendidos:

* /vision/line_finder/find_table_edge (vision_msgs/FindLines): Servicio para encontrar el borde de una mesa. Se busca un plano horizontal, luego se detectan bordes y finalmente se aplica T Hough. Se publica el tópico /vision/obj_reco/markers con la línea resultante. 
* /vision/line_finder/find_horizontal_plane_ransac (vision_msgs/FindPlanes): Encuentra un plano utilizando RANSAC. Primero se filtra la nube por distancia, luego se calculan normales. Con los pixeles cuyas normales son casi verticales (plano horizontal) se encuentra el primer plano que cumpla con el tamaño y número de puntos mínimos, utilizando Ransac y PCA.
* /vision/obj_reco/detect_and_recognize_objects (vision_msgs/RecognizeObjects): Filtra la nube de puntos por distancia, encuentra un plano horizontal, se queda con los puntos por encima del plano, los agrupa mediante bordes y luego hace un reconocimiento comparando histogramas del canal H del objeto en HSV. Se devuelve un arreglo con todos los objetos encontrados. 
* /vision/obj_reco/detect_and_recognize_object (vision_msgs/RecognizeObject): Lo mismo que el anterior, pero devuelve false si no se encuentra el objeto específico requerido. No devuelve un arreglo, sino true/false dependiendo de si se encontró el objeto particular o no. 
* /vision/obj_reco/recognize_objects (vision_msgs/RecognizeObjects): Hace lo mismo que el anterior pero supone que el objeto ya viene segmentado, es decir, no hace segmentación de plano ni clustering. Supone que ya solo se tiene que hacer la comparación de color y devuelve el objeto reconocido.
* /vision/obj_reco/detect_and_train_object (vision_msgs/TrainObject): Hace todo el proceso de segmentación (remover plano y agrupar). Debe haber un solo objeto sobre el plano. Se guarda una imagen del objeto segmentado en la carpeta de imágenes de entrenamiento, dada por el parámetro ~training_dir, en una subcarpeta con el nombre del objeto. Cuando el programa reinicia, entrena los histogramas con todas las imágenes de cada carpeta en la carpeta ~training_dir. 
* /vision/get_points_above_plane (vision_msgs/PreprocessPointCloud): Devuelve una PointCloud2 con los puntos que están arriba de un plano horizontal. La nube mide lo mismo que la original y solo se colocan en cero y en negro todos los puntos que no están por encima del plano encontrado, o que están fuera del bounding box para el filtrado por distancia. 