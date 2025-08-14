// traffic_omp.c
// Simulación de tráfico con semáforos y vehículos (Versión Paralela con OpenMP - OPTIMIZADA: Paso 7)

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>
#include <omp.h>

typedef enum { RED = 0, GREEN = 1, YELLOW = 2 } LightState;

// ----------------------- Estructuras -----------------------
typedef struct {
    int        id;
    LightState state;
    double     time_in_state; // segundos en el estado actual
    double     t_green;       // duración de verde (<= 10 s)
    double     t_yellow;      // duración de amarillo (<= 10 s)
    double     t_red;         // duración de rojo (<= 10 s)
} TrafficLight;

typedef struct {
    int    id;
    int    lane;           // 0..3 (N, E, S, O)
    double pos;            // distancia a la línea de alto (m)
    double speed;          // m/s
    bool   waiting;        // está detenido
    double total_wait;     // s acumulados esperando
    int    crossings;      // 0 o 1 (cruzó)
    bool   finished;       // true cuando ya cruzó
} Vehicle;

typedef struct {
    int           num_lanes;
    int           num_lights;
    double        stop_distance;
    TrafficLight* lights;
} Intersection;

// ----------------------- Utilidades -----------------------
static inline double rand_uniform(double a, double b) { return a + (b - a) * (rand() / (double)RAND_MAX); }

static inline const char* state_to_str(LightState s) {
    return (s == GREEN) ? "V" : (s == YELLOW ? "A" : "R");
}

// ----------------------- Semáforos -----------------------
static inline void update_traffic_light(TrafficLight* L, double dt) {
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

// ----------------------- Vehículos -----------------------
// Devuelve 1 si el vehículo cruzó en este paso; 0 si no.
static inline int move_vehicle(Vehicle* V, const Intersection* X, double dt) {
    if (V->finished) return 0; // ya cruzó

    TrafficLight* L = &X->lights[V->lane];

    // Si está esperando y la luz permite, sale
    if (V->waiting && (L->state == GREEN || L->state == YELLOW)) {
        V->waiting = false;
    }

    // Avance simple si no espera
    if (!V->waiting) {
        V->pos -= V->speed * dt;
    }

    // Llegó a la línea de alto
    if (V->pos <= 0.0) {
        if (L->state == GREEN || L->state == YELLOW) {
            V->crossings = 1;
            V->finished = true;
            V->pos = 0.0;
            return 1;
        } else { // ROJO: se detiene a cierta distancia antes de la línea
            V->pos = X->stop_distance;
            V->waiting = true;
        }
    }

    if (V->waiting) V->total_wait += dt;
    return 0;
}

// ----------------------- Inicialización -----------------------
void init_intersection(Intersection* X, int num_lanes) {
    X->num_lanes = num_lanes;
    X->num_lights = num_lanes;
    X->stop_distance = 2.0;
    X->lights = (TrafficLight*)calloc(X->num_lights, sizeof(TrafficLight));

    for (int i = 0; i < X->num_lights; ++i) {
        X->lights[i].id = i;
        // Tiempos distintos por semáforo (<= 10 s) para ver cambios frecuentes
        X->lights[i].t_green  = rand_uniform(5.0, 9.0); // 5–9 s
        X->lights[i].t_yellow = rand_uniform(2.0, 4.0); // 2–4 s
        X->lights[i].t_red    = rand_uniform(5.0, 9.0); // 5–9 s
        X->lights[i].state = (i % 2 == 0) ? GREEN : RED;
        X->lights[i].time_in_state = 0.0;
    }
}

Vehicle* init_vehicles(int N) {
    Vehicle* A = (Vehicle*)calloc(N, sizeof(Vehicle));
    for (int i = 0; i < N; ++i) {
        A[i].id = i;
        A[i].lane = i % 4;
        A[i].pos = rand_uniform(20.0, 200.0);
        A[i].speed = rand_uniform(6.0, 14.0);
        A[i].waiting = false;
        A[i].total_wait = 0.0;
        A[i].crossings = 0;
        A[i].finished = false;
    }
    return A;
}

// ----------------------- Impresión amigable -----------------------
void print_configuration(const Vehicle* V, int N, const Intersection* X) {
    printf("\nResumen de configuración:\n");
    for (int i = 0; i < N; ++i) {
        printf("Vehículo %d - Carril: %d, Velocidad: %.2f m/s, Posición inicial: %.2f m\n",
               V[i].id, V[i].lane, V[i].speed, V[i].pos);
    }
    for (int i = 0; i < X->num_lights; ++i) {
        printf("Semáforo %d - Estado inicial: %s, Tiempos: R: %.1fs, V: %.1fs, A: %.1fs\n",
               i, state_to_str(X->lights[i].state), X->lights[i].t_red, X->lights[i].t_green, X->lights[i].t_yellow);
    }
    printf("\n");
}

void print_state(int step, double sim_time, const Vehicle* V, const int* crossed_now, int N, const Intersection* X) {
    printf("Iteración %d (t=%.1fs):\n", step, sim_time);
    for (int i = 0; i < N; ++i) {
        if (crossed_now && crossed_now[i]) {
            printf("Vehículo %d - Carril: %d, Posición: 0.00 (CRUZÓ en esta iteración)\n",
                   V[i].id, V[i].lane);
        } else if (V[i].finished) {
            printf("Vehículo %d - Carril: %d, Posición: 0.00 (YA CRUZÓ)\n",
                   V[i].id, V[i].lane);
        } else {
            printf("Vehículo %d - Carril: %d, Posición: %.2f%s\n",
                   V[i].id, V[i].lane, V[i].pos, V[i].waiting ? " (ESPERANDO)" : "");
        }
    }
    for (int i = 0; i < X->num_lights; ++i) {
        printf("Semáforo %d - Estado: %s, Tiempo en estado: %.1fs\n",
               i, state_to_str(X->lights[i].state), X->lights[i].time_in_state);
    }
    printf("\n");
}

// ----------------------- Simulación  -----------------------
void run_simulation(int num_vehicles, int print_every, double dt, unsigned int seed) {
    srand(seed);

    double wall_t0 = omp_get_wtime(); // inicio medición wall-clock de alta precisión

    Intersection X;
    init_intersection(&X, 4);
    Vehicle* V = init_vehicles(num_vehicles);

    // Resumen de configuración
    print_configuration(V, num_vehicles, &X);

    int total_crossed = 0;
    int step = 0;
    double sim_time = 0.0;
    int* crossed_now = (int*)calloc(num_vehicles, sizeof(int));
    int crossed_step = 0;

    // Equipo estable: sin cambios de tamaño por iteración (reduce overhead)
    omp_set_dynamic(0);

    // Región paralela PERSISTENTE: todos los hilos permanecen vivos durante toda la simulación.
    #pragma omp parallel default(shared)
    {
        for (;;) {

            // --- Salida temprana sincronizada ---
            #pragma omp barrier
            if (omp_get_thread_num() == 0 && total_crossed >= num_vehicles) {
                // listo para salir
            }
            #pragma omp barrier
            if (total_crossed >= num_vehicles) break;

            // --- Un solo hilo: actualizar semáforos y limpiar eventos ---
            #pragma omp single
            {
                for (int i = 0; i < X.num_lights; ++i) {
                    update_traffic_light(&X.lights[i], dt); // bucle pequeño: secuencial para evitar overhead
                }
                for (int i = 0; i < num_vehicles; ++i) crossed_now[i] = 0;

                crossed_step = 0;
            }

            // --- Paralelo: mover vehículos (trabajo dominante) ---
            #pragma omp for schedule(static) reduction(+:crossed_step)
            for (int i = 0; i < num_vehicles; ++i) {
                int c = move_vehicle(&V[i], &X, dt);
                if (c) crossed_now[i] = 1;
                crossed_step += c;
            }

            // --- Un solo hilo: acumular totales y snapshot ---
            #pragma omp single
            {
                total_crossed += crossed_step;
                step += 1;
                sim_time += dt;

                if (print_every > 0 && (step % print_every) == 0) {
                    print_state(step, sim_time, V, crossed_now, num_vehicles, &X);
                    // printf("Hilos del equipo: %d\n\n", omp_get_num_threads());
                }
            }
        } // fin for(;;)
    } // fin región paralela

    double wall_t1 = omp_get_wtime(); // fin medición

    // Métricas finales
    double avg_wait = 0.0;
    int total_crossings = 0;
    for (int i = 0; i < num_vehicles; ++i) {
        avg_wait += V[i].total_wait;
        total_crossings += V[i].crossings; // 0 o 1
    }
    avg_wait /= (double)num_vehicles;

    printf("\n--- Resumen (OpenMP OPTIMIZADO) ---\n");
    printf("Vehículos: %d, Pasos ejecutados: %d, dt=%.1f s\n", num_vehicles, step, dt);
    printf("Vehículos que cruzaron: %d/%d\n", total_crossed, num_vehicles);
    printf("Cruces totales por vehículo (suma de V.crossings): %d\n", total_crossings);
    printf("Espera promedio por vehículo: %.3f s\n", avg_wait);
    printf("Tiempo total SIMULADO: %.1f s\n", sim_time);
    printf("Tiempo de EJECUCIÓN (wall clock): %.6f s\n", wall_t1 - wall_t0);

    free(crossed_now);
    free(X.lights);
    free(V);
}

int main(int argc, char** argv) {
    int    N           = (argc > 1) ? atoi(argv[1]) : 200; // vehículos
    int    print_every = (argc > 2) ? atoi(argv[2]) : 5;   // imprimir cada k pasos (= k segundos)
    double dt          = 1.0;                              // s
    unsigned int seed  = (argc > 3) ? (unsigned int)atoi(argv[3]) : (unsigned int)time(NULL);

    printf("OpenMP: max threads disponibles: %d\n", omp_get_max_threads());
    run_simulation(N, print_every, dt, seed);
    return 0;
}
