// traffic_seq.c
// Simulación simple de tráfico con semáforos y vehículos (Versión Secuencial)

//
// Notas de diseño (resumen de los pasos solicitados):
// 1) Estructuras: Vehicle, TrafficLight, Intersection.
// 2) Inicialización: creación de N vehículos, semáforos con ciclo (G/Y/R).
// 3) Semáforos: lógica de cambio de estado con temporizador.
// 4) Vehículos: movimiento simplificado según el estado del semáforo del carril.
// 5) Bucle principal: en cada iteración actualiza semáforos y mueve vehículos.
// 6) (Secuencial) No hay hilos; la versión paralela ajusta hilos dinámicamente.
//
// Simplificaciones importantes para mantener el laboratorio manejable:
//  - 4 carriles/lados (0:N, 1:E, 2:S, 3:O). Cada carril tiene un semáforo propio.
//  - La posición del vehículo es su distancia a la línea de alto (m).
//    Si pos <= 0 y la luz permite paso (VERDE/AMARILLO), se considera que "cruza"
//    y se reubica el vehículo a una distancia aleatoria para simular flujo.
//  - No se modelan colisiones ni interacción entre vehículos (modelo libre).
//  - No hay modelos de aceleración; velocidad constante por vehículo.
//  - Este código es didáctico y fácil de extender para el Paso 7 (pruebas/optimización).

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>

typedef enum { RED = 0, GREEN = 1, YELLOW = 2 } LightState;

typedef struct {
    int        id;
    LightState state;
    double     time_in_state; // segundos acumulados en el estado actual
    double     t_green;       // duración de verde
    double     t_yellow;      // duración de amarillo
    double     t_red;         // duración de rojo
} TrafficLight;

typedef struct {
    int    id;
    int    lane;           // 0..3 (N, E, S, O)
    double pos;            // distancia a la línea de alto (m)
    double speed;          // m/s (constante en este modelo)
    bool   waiting;        // si está detenido esperando verde
    double total_wait;     // tiempo total esperando (métrica)
    int    crossings;      // veces que cruzó la intersección
} Vehicle;

typedef struct {
    int           num_lanes;    //número de carriles
    int           num_lights;   //número de semáforos
    double        stop_distance; // distancia mínima a la línea donde se detiene si está en rojo
    TrafficLight* lights;        // arreglo de num_lights (uno por carril)
} Intersection;

// ---------- Utilidades ----------
static inline double rand_uniform(double a, double b) {
    return a + (b - a) * (rand() / (double)RAND_MAX);
}

// ---------- Semáforos ----------
void update_traffic_light(TrafficLight* L, double dt) {
    L->time_in_state += dt;
    switch (L->state) {
        case GREEN:
            if (L->time_in_state >= L->t_green) {
                L->state = YELLOW;
                L->time_in_state = 0.0;
            }
            break;
        case YELLOW:
            if (L->time_in_state >= L->t_yellow) {
                L->state = RED;
                L->time_in_state = 0.0;
            }
            break;
        case RED:
        default:
            if (L->time_in_state >= L->t_red) {
                L->state = GREEN;
                L->time_in_state = 0.0;
            }
            break;
    }
}

// ---------- Vehículos ----------
// Devuelve 1 si el vehículo cruzó en este paso; 0 si no.
int move_vehicle(Vehicle* V, const Intersection* X, double dt) {
    TrafficLight* L = &X->lights[V->lane];

    // Si está esperando en la línea y la luz permite avanzar, sale.
    if (V->waiting && (L->state == GREEN || L->state == YELLOW)) {
        V->waiting = false;
    }

    // Movimiento simple si no está esperando
    if (!V->waiting) {
        V->pos -= V->speed * dt;
    }

    // Lógica al llegar a la línea de alto (pos <= 0)
    if (V->pos <= 0.0) {
        if (L->state == GREEN || L->state == YELLOW) {
            // Cruza la intersección (éxito) y se reubica para simular llegada de nuevo flujo
            V->crossings += 1;
            // Reaparece a una distancia aleatoria (ej. entre 60 y 200 m)
            V->pos = rand_uniform(60.0, 200.0);
            // Mantiene su velocidad, carril y demás estados
            return 1;
        } else { // ROJO
            // Se detiene justo antes de la línea
            V->pos = X->stop_distance;
            V->waiting = true;
        }
    }

    // Si está esperando, acumula tiempo de espera
    if (V->waiting) {
        V->total_wait += dt;
    }

    return 0;
}

// ---------- Inicialización ----------
void init_intersection(Intersection* X, int num_lanes) {
    X->num_lanes = num_lanes;
    X->num_lights = num_lanes;
    X->stop_distance = 2.0; // 2 metros antes de la línea
    X->lights = (TrafficLight*)calloc(X->num_lights, sizeof(TrafficLight));

    // Ejemplo: ciclos idénticos pero desfasables si se desea
    for (int i = 0; i < X->num_lights; ++i) {
        X->lights[i].id = i;
        X->lights[i].t_green  = 12.0;
        X->lights[i].t_yellow = 3.0;
        X->lights[i].t_red    = 15.0;
        X->lights[i].state = (i % 2 == 0) ? GREEN : RED; // alternados para comenzar
        X->lights[i].time_in_state = 0.0;
    }
}

Vehicle* init_vehicles(int N) {
    Vehicle* A = (Vehicle*)calloc(N, sizeof(Vehicle));
    for (int i = 0; i < N; ++i) {
        A[i].id = i;
        A[i].lane = i % 4; // distribuir en carriles 0..3
        A[i].pos = rand_uniform(20.0, 200.0); // a cierta distancia
        A[i].speed = rand_uniform(6.0, 14.0); // m/s (~22-50 km/h)
        A[i].waiting = false;
        A[i].total_wait = 0.0;
        A[i].crossings = 0;
    }
    return A;
}

// ---------- Simulación ----------
void run_simulation(int num_vehicles, int steps, double dt, unsigned int seed) {
    srand(seed);

    Intersection X;
    init_intersection(&X, 4);
    Vehicle* V = init_vehicles(num_vehicles);

    int total_crossed = 0;

    for (int t = 0; t < steps; ++t) {
        // 1) Actualizar semáforos
        for (int i = 0; i < X.num_lights; ++i) {
            update_traffic_light(&X.lights[i], dt);
        }
        // 2) Mover vehículos
        int crossed_step = 0;
        for (int i = 0; i < num_vehicles; ++i) {
            crossed_step += move_vehicle(&V[i], &X, dt);
        }
        total_crossed += crossed_step;

        // Salida periódica para seguimiento (cada 10 pasos)
        if ((t % 10) == 0) {
            int reds = 0, greens = 0, yellows = 0;
            for (int i = 0; i < X.num_lights; ++i) {
                if (X.lights[i].state == RED) reds++;
                else if (X.lights[i].state == GREEN) greens++;
                else yellows++;
            }
            printf("[t=%3d s] Cruces=%d (acum=%d) | Semáforos -> G:%d Y:%d R:%d\n",
                   t, crossed_step, total_crossed, greens, yellows, reds);
        }
    }

    // Métricas simples al final
    double avg_wait = 0.0;
    int total_crossings = 0;
    for (int i = 0; i < num_vehicles; ++i) {
        avg_wait += V[i].total_wait;
        total_crossings += V[i].crossings;
    }
    avg_wait /= (double)num_vehicles;
    printf("\n--- Resumen (Secuencial) ---\n");
    printf("Vehículos: %d, Pasos: %d, dt=%.1f s\n", num_vehicles, steps, dt);
    printf("Cruces totales contados por iteración: %d\n", total_crossed);
    printf("Cruces totales por vehículo (suma de V.crossings): %d\n", total_crossings);
    printf("Espera promedio por vehículo: %.2f s\n", avg_wait);

    free(X.lights);
    free(V);
}

int main(int argc, char** argv) {
    int    N     = (argc > 1) ? atoi(argv[1]) : 60;   // vehículos
    int    steps = (argc > 2) ? atoi(argv[2]) : 120;  // iteraciones
    double dt    = 1.0;                                // s
    unsigned int seed = (argc > 3) ? (unsigned int)atoi(argv[3]) : (unsigned int)time(NULL);

    run_simulation(N, steps, dt, seed);
    return 0;
}
