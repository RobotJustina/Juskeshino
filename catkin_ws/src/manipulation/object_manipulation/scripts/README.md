# Nodo grasp_node


### Funciones
generates_candidates(grip_point , obj_pose, rotacion, obj_state , name_frame, step, num_candidates, type_obj = None, offset = 0):    

        grip_point:     Vector que contiene  el punto origen de los sistemas candidatos, entra en el frame 'object'.
        obj_pose:       Orientacion del objeto en msg Pose en frame 'object', expresado en cuaterniones.
        rotacion:       Es un string: roll , pitch o yaw: 'R', 'P', 'Y', tipo de rotacion a realizar en el object_frame.
        obj_state:      Es un string que  indica si el objeto esta 'horizontal' o 'vertical'.
        object_frame:   Es el frame en el cual se van a generar los candidatos de retorno.
        step:           Grados de distancia entre un candidato y otro, pueden ser negativos o positivos.
        num_candidates: Es la cantidad de candidatos que se desea generar.

        Retorna una lista de poses candidatas expresadas en cuaterniones (msg Pose)

### Tópicos suscritos:



### Tópicos publicados:

### Servicios requeridos:


### Servicios atendidos:

