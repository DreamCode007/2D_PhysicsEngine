#include "GPUPhysicsWorld.h"
#include <Python.h>
#include <iostream>
#include <cstring>
#include <random>

struct GPUPhysicsWorld::Impl {
    PyObject* pModule = nullptr;
    PyObject* pEngine = nullptr;
    PyObject* pStepFunc = nullptr;
    PyObject* pGetPosFunc = nullptr;
    PyObject* pGetAngFunc = nullptr;
    PyObject* pGetShapesFunc = nullptr;
    PyObject* pGetExtentsFunc = nullptr;
    PyObject* pSpawnFunc = nullptr;
    PyObject* pSetPosFunc = nullptr;
    PyObject* pSetVelFunc = nullptr;

    int bodyCount = 0;
    float worldWidth = 1080.0f;
    float groundLevel = 650.0f;
    float gravity = 500.0f;

    std::vector<float> positions;   // [2 * bodyCount]
    std::vector<float> angles;      // [bodyCount]
    std::vector<int>   shapeTypes;  // [bodyCount]
    std::vector<float> halfExtents; // [2 * bodyCount]

    Impl() {
        if (!Py_IsInitialized()) {
            Py_SetPythonHome(L"C:\\Users\\JUNAID\\AppData\\Local\\Programs\\Python\\Python310");
            Py_Initialize();
        }

        PyRun_SimpleString(
            "import sys\n"
            "sys.path.insert(0, 'c:/Users/JUNAID/Desktop/PhysicsEngine/Python/gpu_physics')\n"
        );

        pModule = PyImport_ImportModule("batch_sat_physics");
        if (!pModule) {
            PyErr_Print();
            std::cerr << "[GPUPhysicsWorld] Error: Failed to import batch_sat_physics!" << std::endl;
        }
    }

    ~Impl() {
        CleanupFuncs();
        Py_XDECREF(pEngine);
        Py_XDECREF(pModule);
    }

    void CleanupFuncs() {
        Py_XDECREF(pStepFunc);
        Py_XDECREF(pGetPosFunc);
        Py_XDECREF(pGetAngFunc);
        Py_XDECREF(pGetShapesFunc);
        Py_XDECREF(pGetExtentsFunc);
        Py_XDECREF(pSpawnFunc);
        Py_XDECREF(pSetPosFunc);
        Py_XDECREF(pSetVelFunc);
        pStepFunc = pGetPosFunc = pGetAngFunc = pGetShapesFunc = pGetExtentsFunc = pSpawnFunc = pSetPosFunc = pSetVelFunc = nullptr;
    }

    void InitEngine(int count, float worldW, float groundY, float gravityY) {
        worldWidth = worldW;
        groundLevel = groundY;
        gravity = gravityY;
        bodyCount = count;

        if (!pModule) return;
        CleanupFuncs();
        Py_XDECREF(pEngine);

        PyObject* pClass = PyObject_GetAttrString(pModule, "GPUSATPhysicsEngine");
        PyObject* pArgs = Py_BuildValue("(fff)", gravity, groundLevel, worldWidth);
        pEngine = PyObject_CallObject(pClass, pArgs);
        Py_DECREF(pArgs);
        Py_DECREF(pClass);

        if (!pEngine) {
            PyErr_Print();
            std::cerr << "[GPUPhysicsWorld] Error: Failed to instantiate GPUSATPhysicsEngine!" << std::endl;
            return;
        }

        pStepFunc       = PyObject_GetAttrString(pEngine, "step");
        pGetPosFunc     = PyObject_GetAttrString(pEngine, "get_positions_numpy");
        pGetAngFunc     = PyObject_GetAttrString(pEngine, "get_angles_numpy");
        pGetShapesFunc  = PyObject_GetAttrString(pEngine, "get_shape_types_numpy");
        pGetExtentsFunc = PyObject_GetAttrString(pEngine, "get_half_extents_numpy");
        pSpawnFunc      = PyObject_GetAttrString(pEngine, "spawn_body");
        pSetPosFunc     = PyObject_GetAttrString(pEngine, "set_body_pos");
        pSetVelFunc     = PyObject_GetAttrString(pEngine, "set_body_vel");

        PopulateInitialBodies(count);
    }

    void PopulateInitialBodies(int count) {
        bodyCount = count;
        std::string initCode = 
            "import numpy as np\n"
            "rng = np.random.default_rng(42)\n"
            "n = " + std::to_string(count) + "\n"
            "pos  = rng.uniform([30, 30], [" + std::to_string(worldWidth - 30) + ", " + std::to_string(groundLevel - 150) + "], (n, 2)).tolist()\n"
            "vel  = rng.uniform([-100, -20], [100, 100], (n, 2)).tolist()\n"
            "ang  = rng.uniform(0, 6.28, n).tolist()\n"
            "ang_v = rng.uniform(-4.0, 4.0, n).tolist()\n"
            "mass = rng.uniform(20, 60, n).tolist()\n"
            "st   = [0 if i % 2 == 0 else 1 for i in range(n)]\n"
            "he   = []\n"
            "for s in st:\n"
            "    if s == 0:\n"
            "        r = float(rng.uniform(8, 16))\n"
            "        he.append([r, r])\n"
            "    else:\n"
            "        he.append([float(rng.uniform(10, 22)), float(rng.uniform(8, 16))])\n"
            "rest = rng.uniform(0.5, 0.8, n).tolist()\n"
            "engine.add_bodies(pos, vel, ang, ang_v, mass, st, he, rest)\n";

        PyObject* pDict = PyModule_GetDict(pModule);
        PyDict_SetItemString(pDict, "engine", pEngine);
        PyRun_String(initCode.c_str(), Py_file_input, pDict, pDict);

        FetchAllState();
    }

    void Step(float dt) {
        if (!pStepFunc || !pEngine) return;

        PyObject* pArgs = Py_BuildValue("(f)", dt);
        PyObject* pRes = PyObject_CallObject(pStepFunc, pArgs);
        Py_DECREF(pArgs);

        if (pRes) {
            Py_DECREF(pRes);
            FetchDynamicState();
        } else {
            PyErr_Print();
        }
    }

    void SpawnBody(float x, float y, float vx, float vy, float angle, float ang_v, float mass, int shapeType, float hw, float hh, float rest) {
        if (!pSpawnFunc) return;
        PyObject* pArgs = Py_BuildValue("(fffffffifff)", x, y, vx, vy, angle, ang_v, mass, shapeType, hw, hh, rest);
        PyObject* pRes = PyObject_CallObject(pSpawnFunc, pArgs);
        Py_DECREF(pArgs);
        if (pRes) {
            Py_DECREF(pRes);
            bodyCount++;
            FetchAllState();
        } else {
            PyErr_Print();
        }
    }

    void SetBodyPosition(int index, float x, float y) {
        if (!pSetPosFunc) return;
        PyObject* pArgs = Py_BuildValue("(iff)", index, x, y);
        PyObject* pRes = PyObject_CallObject(pSetPosFunc, pArgs);
        Py_DECREF(pArgs);
        if (pRes) Py_DECREF(pRes);
        else PyErr_Print();
    }

    void SetBodyVelocity(int index, float vx, float vy) {
        if (!pSetVelFunc) return;
        PyObject* pArgs = Py_BuildValue("(iff)", index, vx, vy);
        PyObject* pRes = PyObject_CallObject(pSetVelFunc, pArgs);
        Py_DECREF(pArgs);
        if (pRes) Py_DECREF(pRes);
        else PyErr_Print();
    }

    void FetchAllState() {
        FetchStaticState();
        FetchDynamicState();
    }

    void FetchStaticState() {
        if (!pEngine) return;
        // Shape types
        if (pGetShapesFunc) {
            PyObject* pArr = PyObject_CallObject(pGetShapesFunc, NULL);
            if (pArr) {
                Py_buffer view;
                if (PyObject_GetBuffer(pArr, &view, PyBUF_SIMPLE) == 0) {
                    shapeTypes.resize(bodyCount);
                    std::memcpy(shapeTypes.data(), view.buf, bodyCount * sizeof(int));
                    PyBuffer_Release(&view);
                }
                Py_DECREF(pArr);
            }
        }
        // Half extents
        if (pGetExtentsFunc) {
            PyObject* pArr = PyObject_CallObject(pGetExtentsFunc, NULL);
            if (pArr) {
                Py_buffer view;
                if (PyObject_GetBuffer(pArr, &view, PyBUF_SIMPLE) == 0) {
                    halfExtents.resize(bodyCount * 2);
                    std::memcpy(halfExtents.data(), view.buf, bodyCount * 2 * sizeof(float));
                    PyBuffer_Release(&view);
                }
                Py_DECREF(pArr);
            }
        }
    }

    void FetchDynamicState() {
        if (!pEngine) return;
        // Positions
        if (pGetPosFunc) {
            PyObject* pArr = PyObject_CallObject(pGetPosFunc, NULL);
            if (pArr) {
                Py_buffer view;
                if (PyObject_GetBuffer(pArr, &view, PyBUF_SIMPLE) == 0) {
                    positions.resize(bodyCount * 2);
                    std::memcpy(positions.data(), view.buf, bodyCount * 2 * sizeof(float));
                    PyBuffer_Release(&view);
                }
                Py_DECREF(pArr);
            }
        }
        // Angles
        if (pGetAngFunc) {
            PyObject* pArr = PyObject_CallObject(pGetAngFunc, NULL);
            if (pArr) {
                Py_buffer view;
                if (PyObject_GetBuffer(pArr, &view, PyBUF_SIMPLE) == 0) {
                    angles.resize(bodyCount);
                    std::memcpy(angles.data(), view.buf, bodyCount * sizeof(float));
                    PyBuffer_Release(&view);
                }
                Py_DECREF(pArr);
            }
        }
    }
};

GPUPhysicsWorld::GPUPhysicsWorld() : impl(std::make_unique<Impl>()) {}
GPUPhysicsWorld::~GPUPhysicsWorld() = default;

void GPUPhysicsWorld::Init(int count, float worldW, float groundY, float gravityY) {
    impl->InitEngine(count, worldW, groundY, gravityY);
}

void GPUPhysicsWorld::Step(float dt) {
    impl->Step(dt);
}

int GPUPhysicsWorld::GetBodyCount() const {
    return impl->bodyCount;
}

const std::vector<float>& GPUPhysicsWorld::GetPositions() const {
    return impl->positions;
}

const std::vector<float>& GPUPhysicsWorld::GetAngles() const {
    return impl->angles;
}

const std::vector<int>& GPUPhysicsWorld::GetShapeTypes() const {
    return impl->shapeTypes;
}

const std::vector<float>& GPUPhysicsWorld::GetHalfExtents() const {
    return impl->halfExtents;
}

void GPUPhysicsWorld::SpawnCircle(float x, float y, float radius, float mass) {
    impl->SpawnBody(x, y, 0.0f, 0.0f, 0.0f, 0.0f, mass, 0, radius, radius, 0.8f);
}

void GPUPhysicsWorld::SpawnBox(float x, float y, float width, float height, float mass) {
    impl->SpawnBody(x, y, 0.0f, 0.0f, 0.0f, 0.0f, mass, 1, width * 0.5f, height * 0.5f, 0.6f);
}

void GPUPhysicsWorld::SetBodyPosition(int index, float x, float y) {
    impl->SetBodyPosition(index, x, y);
}

void GPUPhysicsWorld::SetBodyVelocity(int index, float vx, float vy) {
    impl->SetBodyVelocity(index, vx, vy);
}

void GPUPhysicsWorld::Reset(int count) {
    impl->PopulateInitialBodies(count);
}
