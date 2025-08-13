// traffic_seq.c
// Simulación simple de tráfico con semáforos y vehículos (Versión Secuencial)
// Pasos implementados: 1 a 6 (el 7 se hará después de pruebas)
// Compilación: gcc -O2 -std=c11 traffic_seq.c -o traffic_seq
// Ejecución de ejemplo:
//   ./traffic_seq 8 60 1        # 8 vehículos, 60 s, imprimir cada 1 s
//   ./traffic_seq 80 120 10     # 80 vehículos, 120 s, imprimir cada 10 s
//
// Cambios solicitados en esta versión:
//  - "Resumen de configuración" al inicio (vehículos y semáforos).
//  - Impresión de "estado de la simulación" cada k segundos (argumento 3).
//  - Cada vehículo solo puede cruzar UNA vez. Tras cruzar, queda "finished" y ya no se mueve ni cuenta espera.
//  - La simulación se DETIENE si ya cruzaron todos los vehículos o si se alcanzó el tiempo límite.
//  - Estadísticas finales ya existentes.
//  - Comentarios claros explicando el código.
//
// Simplificaciones importantes (modelo didáctico):
//  - 4 carriles/lados (0:N, 1:E, 2:S, 3:O). Cada carril tiene su semáforo.
//  - La posición del vehículo es su distancia a la línea de alto (m).
//  - Si pos <= 0 y la luz permite paso (VERDE/AMARILLO), cruza y finaliza su participación.
//  - No se modelan colisiones ni interacción entre vehículos; velocidad constante por vehículo.

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>

typedef enum { RED = 0, GREEN = 1, YELLOW = 2 } LightState;

// ----------------------- Estructuras (Paso 1) -----------------------
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
    double total_wait;     // s acumulados esperando
    int    crossings;      // 0 o 1 (cruzó)
    bool   finished;       // true cuando ya cruzó
} Vehicle;

typedef struct {
    int           num_lanes;     // número de carriles
    int           num_lights;    // número de semáforos
    double        stop_distance; // distancia de detención si hay rojo
    TrafficLight* lights;        // arreglo de num_lights (uno por carril)
} Intersection;

// ----------------------- Utilidades -----------------------
static inline double rand_uniform(double a, double b) { return a + (b - a) * (rand() / (double)RAND_MAX); }

static inline const char* state_to_str(LightState s) {
    return (s == GREEN) ? "V" : (s == YELLOW ? "A" : "R");
}

// ----------------------- Semáforos (Paso 3) -----------------------
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

// ----------------------- Vehículos (Paso 4) -----------------------
// Devuelve 1 si el vehículo cruzó en este paso; 0 si no.
int move_vehicle(Vehicle* V, const Intersection* X, double dt) {
    if (V->finished) return 0; // Ya cruzó en pasos previos

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
            // Cruza la intersección (finaliza)
            V->crossings = 1;
            V->finished = true;
            V->pos = 0.0; // dejar en 0 para imprimir "cruzó"
            return 1;
        } else { // ROJO
            // Se detiene justo antes de la línea
            V->pos = X->stop_distance;
            V->waiting = true;
        }
    }

    // Si está esperando, acumula tiempo de espera
    if (V->waiting) V->total_wait += dt;

    return 0;
}

// ----------------------- Inicialización (Paso 2) -----------------------
void init_intersection(Intersection* X, int num_lanes) {
    X->num_lanes = num_lanes;
    X->num_lights = num_lanes;
    X->stop_distance = 2.0; // 2 metros antes de la línea
    X->lights = (TrafficLight*)calloc(X->num_lights, sizeof(TrafficLight));

    // Ciclos idénticos y estados alternados para comenzar
    for (int i = 0; i < X->num_lights; ++i) {
        X->lights[i].id = i;
        X->lights[i].t_green  = 12.0;
        X->lights[i].t_yellow = 3.0;
        X->lights[i].t_red    = 15.0;
        X->lights[i].state = (i % 2 == 0) ? GREEN : RED; // alternados
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
        printf("Semáforo %d - Estado inicial: %s, Tiempos: R: %.0fs, V: %.0fs, A: %.0fs\n",
               i, state_to_str(X->lights[i].state), X->lights[i].t_red, X->lights[i].t_green, X->lights[i].t_yellow);
    }
    printf("\n");
}

void print_state(int step, double sim_time, const Vehicle* V, const int* crossed_now, int N, const Intersection* X) {
    printf("Iteración %d (t=%.0fs):\n", step, sim_time);
    for (int i = 0; i < N; ++i) {
        if (crossed_now && crossed_now[i]) {
            printf("Vehículo %d - Carril: %d, Posición: 0 (CRUZÓ en esta iteración)\n",
                   V[i].id, V[i].lane);
        } else if (V[i].finished) {
            printf("Vehículo %d - Carril: %d, Posición: 0 (YA CRUZÓ)\n",
                   V[i].id, V[i].lane);
        } else {
            printf("Vehículo %d - Carril: %d, Posición: %.2f%s\n",
                   V[i].id, V[i].lane, V[i].pos, V[i].waiting ? " (ESPERANDO)" : "");
        }
    }
    for (int i = 0; i < X->num_lights; ++i) {
        printf("Semáforo %d - Estado: %s, Tiempo en estado: %.0fs\n",
               i, state_to_str(X->lights[i].state), X->lights[i].time_in_state);
    }
    printf("\n");
}

// ----------------------- Simulación (Paso 5-6) -----------------------
void run_simulation(int num_vehicles, int steps, int print_every, double dt, unsigned int seed) {
    srand(seed);

    Intersection X;
    init_intersection(&X, 4);
    Vehicle* V = init_vehicles(num_vehicles);

    // Mostrar resumen de configuración antes de iniciar
    print_configuration(V, num_vehicles, &X);

    int total_crossed = 0; // cruces acumulados (también = # vehículos finalizados)
    int* crossed_now = (int*)calloc(num_vehicles, sizeof(int)); // eventos de cruce por iteración

    for (int t = 0; t < steps; ++t) {
        double sim_time = (t * dt);

        // 1) Actualizar semáforos
        for (int i = 0; i < X.num_lights; ++i) {
            update_traffic_light(&X.lights[i], dt);
        }

        // Limpiar eventos de esta iteración
        for (int i = 0; i < num_vehicles; ++i) crossed_now[i] = 0;

        // 2) Mover vehículos
        int crossed_step = 0;
        for (int i = 0; i < num_vehicles; ++i) {
            int c = move_vehicle(&V[i], &X, dt);
            if (c) { crossed_now[i] = 1; crossed_step += 1; }
        }
        total_crossed += crossed_step;

        // 3) Impresión según intervalo deseado por el usuario
        if (print_every > 0 && (t % print_every) == 0) {
            print_state(t, sim_time + dt, V, crossed_now, num_vehicles, &X);
        }

        // 4) Criterio de paro anticipado si ya cruzaron todos
        if (total_crossed >= num_vehicles) {
            printf("Todos los vehículos han cruzado. Fin anticipado de la simulación en t=%.0fs.\n\n", sim_time + dt);
            break;
        }
    }

    // Métricas simples al final
    double avg_wait = 0.0;
    int total_crossings = 0;
    for (int i = 0; i < num_vehicles; ++i) {
        avg_wait += V[i].total_wait;
        total_crossings += V[i].crossings; // 0 o 1 por vehículo
    }
    avg_wait /= (double)num_vehicles;
    printf("\n--- Resumen (Secuencial) ---\n");
    printf("Vehículos: %d, Pasos ejecutados (máx): %d, dt=%.1f s\n", num_vehicles, steps, dt);
    printf("Vehículos que cruzaron: %d/%d\n", total_crossed, num_vehicles);
    printf("Cruces totales por vehículo (suma de V.crossings): %d\n", total_crossings);
    printf("Espera promedio por vehículo: %.2f s\n", avg_wait);

    free(crossed_now);
    free(X.lights);
    free(V);
}

int main(int argc, char** argv) {
    int    N           = (argc > 1) ? atoi(argv[1]) : 8;    // vehículos
    int    steps       = (argc > 2) ? atoi(argv[2]) : 60;   // duración (en pasos de dt)
    int    print_every = (argc > 3) ? atoi(argv[3]) : 1;    // imprimir cada k pasos (k*dt segundos)
    double dt          = 1.0;                               // s
    unsigned int seed  = (argc > 4) ? (unsigned int)atoi(argv[4]) : (unsigned int)time(NULL);

    run_simulation(N, steps, print_every, dt, seed);
    return 0;
}
