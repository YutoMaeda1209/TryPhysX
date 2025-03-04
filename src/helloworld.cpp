#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include "PxPhysicsAPI.h"

#pragma comment(lib, "PhysX_64.lib")
#pragma comment(lib, "PhysXCommon_64.lib")
#pragma comment(lib, "PhysXCooking_64.lib")
#pragma comment(lib, "PhysXExtensions_static_64.lib")
#pragma comment(lib, "PhysXFoundation_64.lib")
#pragma comment(lib, "PhysXPvdSDK_static_64.lib")
#pragma comment(lib, "PhysXTask_static_64.lib")
#pragma comment(lib, "SceneQuery_static_64.lib")
#pragma comment(lib, "SimulationController_static_64.lib")

std::atomic<bool> stopFlag(false);

void checkInput()
{
    char ch;
    while (true)
    {
        std::cin >> ch;
        if (ch == 'q')
        { // 'q'が押されたらフラグを立てる
            stopFlag = true;
            break;
        }
    }
}

int main()
{
    // PhysX内で利用するアロケーター
    physx::PxDefaultAllocator m_defaultAllocator;
    // エラー時用のコールバックでエラー内容が入ってる
    physx::PxDefaultErrorCallback m_defaultErrorCallback;
    // 上位レベルのSDK(PxPhysicsなど)をインスタンス化する際に必要
    physx::PxFoundation *m_pFoundation = nullptr;
    // 実際に物理演算を行う
    physx::PxPhysics *m_pPhysics = nullptr;
    // シミュレーションをどう処理するかの設定でマルチスレッドの設定もできる
    physx::PxDefaultCpuDispatcher *m_pDispatcher = nullptr;
    // シミュレーションする空間の単位でActorの追加などもここで行う
    physx::PxScene *m_pScene = nullptr;
    // PVDと通信する際に必要
    physx::PxPvd *m_pPvd = nullptr;

    // Foundationのインスタンス化
    if (m_pFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, m_defaultAllocator, m_defaultErrorCallback), !m_pFoundation)
    {
        return false;
    }
    // PVDと接続する設定
    if (m_pPvd = physx::PxCreatePvd(*m_pFoundation), m_pPvd)
    {
        // PVD側のデフォルトポートは5425
        physx::PxPvdTransport *transport = physx::PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 10);
        m_pPvd->connect(*transport, physx::PxPvdInstrumentationFlag::eALL);
    }
    // Physicsのインスタンス化
    if (m_pPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *m_pFoundation, physx::PxTolerancesScale(), true, m_pPvd), !m_pPhysics)
    {
        return false;
    }
    // 拡張機能用
    if (!PxInitExtensions(*m_pPhysics, m_pPvd))
    {
        return false;
    }
    // 処理に使うスレッドを指定する
    m_pDispatcher = physx::PxDefaultCpuDispatcherCreate(8);
    // 空間の設定
    physx::PxSceneDesc scene_desc(m_pPhysics->getTolerancesScale());
    scene_desc.gravity = physx::PxVec3(0, -9, 0);
    scene_desc.filterShader = physx::PxDefaultSimulationFilterShader;
    scene_desc.cpuDispatcher = m_pDispatcher;
    // 空間のインスタンス化
    m_pScene = m_pPhysics->createScene(scene_desc);
    // PVDの表示設定
    physx::PxPvdSceneClient *pvd_client;
    if (pvd_client = m_pScene->getScenePvdClient(), pvd_client)
    {
        pvd_client->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
        pvd_client->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
        pvd_client->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
    }

    // 平面を追加
    m_pScene->addActor(*physx::PxCreatePlane(
        *m_pPhysics, physx::PxPlane(0, 1, 0, 0),
        *m_pPhysics->createMaterial(0.5f, 0.5f, 0.5f)));

    // 動かすことのできる(動的)剛体を作成
    physx::PxRigidDynamic *rigid_dynamic = m_pPhysics->createRigidDynamic(physx::PxTransform(physx::PxIdentity));
    // 形状(Box)を作成
    physx::PxShape *box_shape = m_pPhysics->createShape(
        // Boxの大きさ
        physx::PxBoxGeometry(1.f, 1.f, 1.f),
        // 摩擦係数と反発係数の設定
        *m_pPhysics->createMaterial(0.5f, 0.5f, 0.5f));
    // 形状を紐づけ
    box_shape->setLocalPose(physx::PxTransform(physx::PxIdentity));
    rigid_dynamic->attachShape(*box_shape);

    // 剛体を空間に追加
    m_pScene->addActor(*rigid_dynamic);

    rigid_dynamic->addForce(physx::PxVec3(0, 10, 0), physx::PxForceMode::eIMPULSE);

    std::thread inputThread(checkInput);
    while (!stopFlag)
    {
        // シミュレーション速度を指定する
        m_pScene->simulate(1.0f / 60.0f);
        // PhysXの処理が終わるまで待つ
        m_pScene->fetchResults(true);
    }

    PxCloseExtensions();
    m_pScene->release();
    m_pDispatcher->release();
    m_pPhysics->release();
    if (m_pPvd)
    {
        m_pPvd->disconnect();
        physx::PxPvdTransport *transport = m_pPvd->getTransport();
        m_pPvd->release();
        transport->release();
    }
    m_pFoundation->release();

    inputThread.join();
    return 0;
}
