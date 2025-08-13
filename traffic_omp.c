// traffic_omp.c
// Simulación simple de tráfico con semáforos y vehículos (Versión Paralela con OpenMP)
// Pasos implementados: 1 a 6 (el 7 se hará después de pruebas)
// Compilación: gcc -O2 -fopenmp -std=c11 traffic_omp.c -o traffic_omp
// Ejecución de ejemplo: OMP_NUM_THREADS=4 ./traffic_omp 80 120
//   -> el programa ajusta dinámicamente hilos en cada iteración (Paso 6).
//
// Paralelismo utilizado (requerido):
//  - #pragma omp parallel for para actualizar múltiples semáforos (Paso 3).
//  - #pragma omp parallel for para mover múltiples vehículos (Paso 4).
//  - Ajuste dinámico de hilos: omp_set_dynamic(1) + omp_set_num_threads(k) por iteración (Paso 6).
//  - Se indica claramente en comentarios qué partes están paralelizadas.
//
// Simplificaciones: mismas que la versión secuencial. Vea traffic_seq.c para detalles.

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>
#include <omp.h>

typedef enum { RED = 0, GREEN = 1, YELLOW = 2 } LightState;

typedef struct {
    int        id;
    LightState state;
    double     time_in_state;   // segundos acumulados en el estado actual
    double     t_green;         // duración de verde
    double     t_yellow;        // duración de amarillo
    double     t_red;           // duración de rojo
} TrafficLight;

typedef struct {
    int    id;
    int    lane;           // 0..3 (N, E, S, O)
    double pos;            // distancia a la línea de alto (m)
    double speed;          // m/s
    bool   waiting;        // si está detenido esperando verde
    double total_wait;     // tiempo total esperando (métrica)
    int    crossings;      // veces que cruzó la intersección
} Vehicle;

typedef struct {
    int           num_lanes;        //número de carriles
    int           num_lights;   //número de semáforos
    double        stop_distance; // distancia mínima a la línea donde se detiene si está en rojo
    TrafficLight* lights;       // arreglo de num_lights (uno por carril
} Intersection;

// ---------- Utilidades ----------
static inline double rand_uniform(double a, double b) {
    return a + (b - a) * (rand() / (double)RAND_MAX);
}

// ---------- Semáforos  ----------
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

// ---------- Vehículos  ----------
// Devuelve 1 si el vehículo cruzó en este paso; 0 si no.
int move_vehicle(Vehicle* V, const Intersection* X, double dt) {
    TrafficLight* L = &X->lights[V->lane];

    if (V->waiting && (L->state == GREEN || L->state == YELLOW)) {
        V->waiting = false;
    }

    if (!V->waiting) {
        V->pos -= V->speed * dt;
    }

    if (V->pos <= 0.0) {
        if (L->state == GREEN || L->state == YELLOW) {
            V->crossings += 1;
            V->pos = rand_uniform(60.0, 200.0);
            return 1;
        } else {
            V->pos = X->stop_distance;
            V->waiting = true;
        }
    }

    if (V->waiting) {
        V->total_wait += dt;
    }

    return 0;
}

// ---------- Inicialización  ----------
void init_intersection(Intersection* X, int num_lanes) {
    X->num_lanes = num_lanes;
    X->num_lights = num_lanes;
    X->stop_distance = 2.0;
    X->lights = (TrafficLight*)calloc(X->num_lights, sizeof(TrafficLight));

    for (int i = 0; i < X->num_lights; ++i) {
        X->lights[i].id = i;
        X->lights[i].t_green  = 12.0;
        X->lights[i].t_yellow = 3.0;
        X->lights[i].t_red    = 15.0;
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
    }
    return A;
}

// ---------- Heurística para ajustar hilos ----------
// Decide cuántos hilos usar en esta iteración según la carga de trabajo.
// - Más vehículos => más hilos.
// - Se limita por omp_get_max_threads() y por un mínimo de 1.
// - También considera cuántos semáforos están en verde (potencial de cruce).
int choose_threads(int num_vehicles, int num_lights_green) {
    int max_threads = omp_get_max_threads();
    // Base en cantidad de vehículos (1 hilo por ~16 vehículos)
    int by_vehicles = (num_vehicles + 15) / 16;
    // Aumentar si hay más luz verde (más trabajo de movimiento/cambios reales)
    int by_greens = (num_lights_green > 0) ? (1 + num_lights_green / 2) : 1;
    int proposed = by_vehicles + by_greens;

    if (proposed < 1) proposed = 1;
    if (proposed > max_threads) proposed = max_threads;
    return proposed;
}

// ---------- Simulación ----------
void run_simulation(int num_vehicles, int steps, double dt, unsigned int seed) {
    srand(seed);

    Intersection X;
    init_intersection(&X, 4);
    Vehicle* V = init_vehicles(num_vehicles);

    int total_crossed = 0;

    // Habilitar ajuste dinámico global (permite al runtime modificar el tamaño real del equipo)
    omp_set_dynamic(1);
    // (Opcional) Habilitar paralelismo anidado si se quisiera anidar regiones.
    omp_set_nested(0);

    for (int t = 0; t < steps; ++t) {
        // Contar luces verdes actuales (para la heurística de hilos)
        int greens_now = 0;
        for (int i = 0; i < X.num_lights; ++i) {
            if (X.lights[i].state == GREEN) greens_now++;
        }

        int threads = choose_threads(num_vehicles, greens_now);
        // Fijar número de hilos "deseado" para las regiones paralelas de esta iteración
        omp_set_num_threads(threads);

        // --- Actualizar semáforos (PARALELIZADO) ---
        // Cada semáforo se actualiza de forma independiente.
        #pragma omp parallel for schedule(static)
        for (int i = 0; i < X.num_lights; ++i) {
            update_traffic_light(&X.lights[i], dt);
        }

        // --- Mover vehículos (PARALELIZADO) ---
        int crossed_step = 0;
        // Reducción para contar cuántos cruzan en este paso
        #pragma omp parallel for reduction(+:crossed_step) schedule(dynamic, 64)
        for (int i = 0; i < num_vehicles; ++i) {
            crossed_step += move_vehicle(&V[i], &X, dt);
        }

        total_crossed += crossed_step;

        if ((t % 10) == 0) {
            int reds = 0, greens = 0, yellows = 0;
            for (int i = 0; i < X.num_lights; ++i) {
                if (X.lights[i].state == RED) reds++;
                else if (X.lights[i].state == GREEN) greens++;
                else yellows++;
            }
            // Nota: threads es el objetivo; el runtime puede decidir menos/más por omp_set_dynamic(1).
            printf("[t=%3d s] Cruces=%d (acum=%d) | Semáforos -> G:%d Y:%d R:%d | hilos objetivo=%d\n",
                   t, crossed_step, total_crossed, greens, yellows, reds, threads);
        }
    }

    double avg_wait = 0.0;
    int total_crossings = 0;
    for (int i = 0; i < num_vehicles; ++i) {
        avg_wait += V[i].total_wait;
        total_crossings += V[i].crossings;
    }
    avg_wait /= (double)num_vehicles;

    printf("\n--- Resumen (OpenMP) ---\n");
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

    // Mensaje inicial sobre el runtime disponible
    printf("OpenMP: max threads disponibles: %d\n", omp_get_max_threads());
    run_simulation(N, steps, dt, seed);
    return 0;
}
