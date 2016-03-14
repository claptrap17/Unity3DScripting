using System;
using PC2D;
using UnityEngine;


[RequireComponent(typeof(BoxCollider2D))]
public class PlatformerMotor2D : MonoBehaviour
{

    public LayerMask staticEnvLayerMask;
    public LayerMask movingPlatformLayerMask;

    public float envCheckDistance = 0.02f;
    public float minDistanceFromEnv = 0.01f;

    public int numOfIterations = 2;

    public bool enableOneWayPlatforms = true;
    public bool oneWayPlatformsAreWalls = true;

    public int additionalRaycastsPerSide = 1;

    public float groundSpeed = 8f;
    public float timeToGroundSpeed = 0.1f;
    public float groundStopDistance = 0.333f;
    public float airSpeed = 5f;


    public bool changeDirectionInAir = true;

    public float timeToAirSpeed = 0.2f;
    public float airStopDistance = 2f;
    public float fallSpeed = 16f;
    public float gravityMultiplier = 4;
    public float fastFallSpeed = 32f;
    public float fastFallGravityMultiplier = 8f;
    public float ladderSpeed = 5f;
    public float jumpHeight = 1.5f;
    public float extraJumpHeight = 1.5f;

    public int numOfAirJumps = 1;

    public float jumpWindowWhenFalling = 0.2f;
    public float jumpWindowWhenActivated = 0.2f;

    public bool enableSlopes = true;

    public float angleAllowedForMoving = 50;
    public float minimumSpeedToMoveUpSlipperySlope = 7.5f;

    public bool changeSpeedOnSlopes = true;

    [Range(0f, 1f)]
    public float speedMultiplierOnSlope = 0.75f;

    public bool stickOnGround = true;

    public float distanceToCheckToStick = 0.4f;

    public bool enableWallJumps = true;

    public float wallJumpMultiplier = 1f;

    [Range(0f, 90f)]
    public float wallJumpAngle = 70;

    public bool enableWallSticks = true;

    public float wallSticksDuration = 1f;

    public bool enableWallSlides = true;

    public float wallSlideSpeed = 5;
    public float timeToWallSlideSpeed = 3;

    public bool enableCornerGrabs = true;

    public float cornerGrabDuration = 1f;
    public float cornerJumpMultiplier = 1f;
    public float cornerDistanceCheck = 0.2f;

    [Range(0.1f, 1f)]
    public float normalizedValidWallInteraction = 0.2f;
    public float ignoreMovementAfterJump = 0.2f;
    public float wallInteractionCooldown = 0.1f;

    [Range(0f, 1f)]
    public float wallInteractionThreshold = 0.5f;

    public bool enableDashes = true;

    public float dashDistance = 3;
    public float dashDuration = 0.2f;
    public float dashCooldown = 0.76f;

    public EasingFunctions.Functions dashEasingFunction = EasingFunctions.Functions.EaseOutQuad;

    public float endDashNoGravityDuration = 0.1f;

    public Action onDash;
    public Action onDashEnd;
    public Action onJump;
    public Action onJumpEnd;
    public Action onAirJump;

    public Action<Vector2> onWallJump;

    public Action onCornerJump;
    public Action onLanded;
    public Action onSlipping;
    public Action onSlippingEnd;

    public Action onDuck;
    public Action onDuckEnd;

    public bool movingPlatformDebug;

    public bool iterationDebug;

    public enum MotorState
    {
        OnGround,
        Jumping,
        Falling,
        Ducking,
        FallingFast,
        WallSliding,
        OnCorner,
        WallSticking,
        Dashing,
        Frozen,
        Slipping,
        FreedomState
    }

    [Flags]
    public enum CollidedSurface
    {
        None = 0x0,
        Ground = 0x1,
        LeftWall = 0x2,
        RightWall = 0x4,
        Ceiling = 0x8,
        SlopeLeft = 0x10,
        SlopeRight = 0x20
    }

    [Flags]
    public enum CollidedArea
    {
        None = 0x0,
        Restricted = 0x01,
        FreedomArea = 0x02,
        Ladder = 0x04 // maybe 0x06
    }

    public enum LadderZone
    {
        Top,
        Bottom,
        Middle
    }

    public LadderZone ladderZone { get; private set; }

    public float normalizedXMovement { get; set; }
    public float normalizedYMovement { get; set; }

    public float timeScale
    {
        get
        {
            return _timeScale;
        }
        set
        {
            if (value > 0)
            {
                if (_timeScale != 0)
                {
                    ReadjustTimers(_timeScale / value);
                }
                else
                {
                    ReadjustTimers(_savedTimeScale / value);
                }
            }

            _savedTimeScale = _timeScale;
            _timeScale = value;

            if (_timeScale < 0)
            {
                _timeScale = 0;
            }
        }
    }

    public Vector2 velocity
    {
        get
        {
            return IsDashing() ? _dashing.dashDir * GetDashSpeed() : _velocity;
        }
        set
        {
            _velocity = value;
        }
    }

    public MotorState motorState { get; private set; }

    public CollidedSurface collidingAgainst { get; private set; }

    public CollidedArea inArea { get; private set; }

    public bool facingLeft { get; set; }

    public Vector2 dashDirection
    {
        get
        {
            return IsDashing() ? _dashing.dashDir : Vector2.zero;
        }
    }

    public float distanceDashed
    {
        get
        {
            return IsDashing() ? _dashing.distanceDashed : 0;
        }
    }

    public float dashDistanceCalculated
    {
        get
        {
            return IsDashing() ? _dashing.distanceCalculated : 0;
        }
    }

    public bool canDash
    {
        get { return _dashing.cooldownFrames < 0; }
    }


    public float amountFastFallen { get; private set; }

    public float amountFallen { get; private set; }

    public float amountJumpedFor { get; private set; }

    public bool fallFast { get; set; }

    public bool jumpingHeld
    {
        get
        {
            return _jumping.held;
        }

        set
        {
            // Since we set held to true on pressed, we only set to false here. This prevent held from being set after a release.
            if (!value)
            {
                _jumping.held = false;
            }

        }
    }

    public bool frozen
    {
        get
        {
            return _frozen;
        }
        set
        {
            if (_frozen != value)
            {
                _frozen = value;

                // do not use ChangeState, because delegates will be called
                if (_frozen)
                {
                    _prevState = motorState;
                    motorState = MotorState.Frozen;
                }
                else
                {
                    motorState = _prevState;
                }
            }
        }
    }

    public MovingPlatformMotor2D connectedPlatform
    {
        get { return _movingPlatformState.platform; }
    }

    public bool onSlope { get; private set; }

    public Vector2 slopeNormal { get; private set; }

    public void Jump()
    {
        _jumping.pressed = true;
        _jumping.timeToldFrames = GetFrameCount(jumpWindowWhenActivated);
        _jumping.height = jumpHeight;

        // Consider jumping held in case there are multiple fixed ticks before the next update tick.
        // This is useful as jumpingHeld may not be set to true with a GetButton() call.
        _jumping.held = true;
    }


    public void Jump(float customHeight)
    {
        Jump();
        _jumping.height = customHeight;
    }

    public void ForceJump()
    {
        Jump();
        _jumping.force = true;
    }

    public void ForceJump(float customHeight)
    {
        ForceJump();
        _jumping.height = customHeight;
    }

    public void EndJump()
    {
        if (IsJumping())
        {
            _jumping.pressed = false;
            _jumping.timeToldFrames = -1;
            _jumping.numAirJumps = 0;
            ChangeState(MotorState.Falling);
        }
    }

    public void ResetAirJump()
    {
        _jumping.numAirJumps = 0;
    }

    public void ResetDashCooldown()
    {
        _dashing.cooldownFrames = -1;
    }

    public void DisconnectFromPlatform()
    {
        _movingPlatformState.platform = null;
    }

    public void Dash()
    {
        _dashing.pressed = true;
        _dashing.dashWithDirection = false;
    }

    public void ForceDash()
    {
        Dash();
        _dashing.force = true;
    }


    public void Dash(Vector2 dir)
    {
        _dashing.pressed = true;
        _dashing.dashWithDirection = true;
        _dashing.dashDir = dir;
    }

    public void ForceDash(Vector2 dir)
    {
        Dash(dir);
        _dashing.force = true;
    }


    public void EndDash()
    {
        // If dashing then end now.
        if (IsDashing())
        {
            _dashing.cooldownFrames = GetFrameCount(dashCooldown);
            _dashing.pressed = false;
            _dashing.gravityEnabledFrames = GetFrameCount(endDashNoGravityDuration);

            _velocity = _dashing.dashDir * GetDashSpeed();

            ChangeState(IsGrounded() ? MotorState.OnGround : MotorState.Falling);
        }
    }



    public void FreedomAreaEnter()
    {
        inArea |= CollidedArea.FreedomArea;
    }

    public bool IsOnFreedomArea()
    {
        return (inArea & CollidedArea.FreedomArea) == CollidedArea.FreedomArea;
    }

    public void FreedomAreaExit()
    {
        inArea &= ~CollidedArea.FreedomArea;
        // leaving a freedom area also implies that cannot be a ladder
        inArea &= ~CollidedArea.Ladder;

        // leave freedom state, and start falling
        if (IsUserHandled())
        {
            FreedomStateExit();
        }
    }


    public void LadderAreaEnter(Bounds area, float topHeight = 0, float bottomHeight = 0)
    {
        inArea |= CollidedArea.Ladder;
        inArea |= CollidedArea.FreedomArea;

        _ladderArea = area;

        // top - 8
        // bottom + 8

        _ladderTopArea = new Bounds(
            new Vector3(area.center.x, area.center.y + area.extents.y - topHeight * 0.5f, 0),
            new Vector3(area.size.x, topHeight, 100)
        );

        _ladderBottomArea = new Bounds(
            new Vector3(area.center.x, area.center.y - area.extents.y + bottomHeight * 0.5f, 0),
            new Vector3(area.size.x, bottomHeight, 100)
        );
    }

    public bool IsOnLadder()
    {
        return (inArea & CollidedArea.Ladder) == CollidedArea.Ladder;
    }

    public void LadderAreaExit()
    {
        inArea &= ~CollidedArea.Ladder;
        // I may leave a freedomArea, i may not... exitFreedomArea must be called

        _ladderBottomArea = new Bounds(Vector3.zero, Vector3.zero);
        _ladderTopArea = new Bounds(Vector3.zero, Vector3.zero);
        _ladderArea = new Bounds(Vector3.zero, Vector3.zero);
    }

    public void SetLadderZone(LadderZone z)
    {
        ladderZone = z;
    }


    public void SetRestrictedArea(Bounds b, bool disableTop = false)
    {
        _restrictedArea = b;

        _restrictedAreaTR = b.center + b.extents;
        _restrictedAreaBL = b.center - b.extents;

        if (disableTop)
        {
            _restrictedAreaTR.y = Mathf.Infinity;
        }
    }

    public void ClearRestrictedArea()
    {
        _restrictedAreaTR.x = Mathf.Infinity;
        _restrictedAreaTR.y = Mathf.Infinity;
        _restrictedAreaBL.x = -Mathf.Infinity;
        _restrictedAreaBL.y = -Mathf.Infinity;
    }


    public void EnableRestrictedArea()
    {
        if (!(_restrictedAreaTR.x == Mathf.Infinity &&
              _restrictedAreaTR.y == Mathf.Infinity &&
              _restrictedAreaBL.x == -Mathf.Infinity &&
              _restrictedAreaBL.y == -Mathf.Infinity))
        {
            inArea |= CollidedArea.Restricted;
        }
    }

    public bool IsRestricted()
    {
        return (inArea & CollidedArea.Restricted) == CollidedArea.Restricted;
    }

    public void DisableRestrictedArea()
    {
        inArea &= ~CollidedArea.Restricted;
    }

    // When enter freedom state velocity is reset, so no inertia
    public void FreedomStateEnter()
    {
        _velocity.y = 0;
        _velocity.x = 0;
        _ignoreGravity = true;
        ChangeState(MotorState.FreedomState);
    }

    public void FreedomStateExit()
    {
        _ignoreGravity = false;
        ChangeState(MotorState.Falling);
    }

    //
    // Quering motor
    //


    public bool IsDashing()
    {
        return motorState == MotorState.Dashing;
    }

    public bool IsDucking()
    {
        return motorState == MotorState.Ducking;
    }

    public bool IsJumping()
    {
        return motorState == MotorState.Jumping;
    }

    public bool IsUserHandled()
    {
        return motorState == MotorState.FreedomState;
    }

    public bool IsFalling()
    {
        return motorState == MotorState.Falling;
    }

    public bool IsFallingFast()
    {
        return motorState == MotorState.FallingFast;
    }

    public bool IsWallSticking()
    {
        return motorState == MotorState.WallSticking;
    }

    public bool IsOnGround()
    {
        return motorState == MotorState.OnGround;
    }

    public bool IsSlipping()
    {
        return motorState == MotorState.Slipping;
    }

    public bool IsOnCorner()
    {
        return motorState == MotorState.OnCorner;
    }

    public bool IsWallSliding()
    {
        return motorState == MotorState.WallSliding;
    }

    public bool IsInAir()
    {
        // TODO dashing on mid-air should be considered?
        return IsJumping() || IsFalling() || IsFallingFast();
    }

    public bool IsOnWall()
    {
        return IsWallSliding() || IsOnCorner() || IsWallSticking();
    }

    public bool IsGrounded()
    {
        return (HasFlag(CollidedSurface.Ground) || onSlope) &&
               !IsJumping() &&
               (onSlope && Vector2.Dot(_velocity, slopeNormal) <= NEAR_ZERO ||
               _velocity.y <= NEAR_ZERO);
    }


    public bool IsForceSlipping()
    {
        return onSlope && Vector3.Dot(Vector3.up, slopeNormal) < _dotAllowedForSlopes;
    }

    public bool IsMovingPlatform(GameObject obj)
    {
        return ((0x1 << obj.layer) & movingPlatformLayerMask) != 0;
    }

    public bool IsStatic(GameObject obj)
    {
        return ((0x1 << obj.layer) & staticEnvLayerMask) != 0;
    }

    private LayerMask _collisionMask;

    private Vector2 _restrictedAreaTR;
    private Bounds _restrictedArea;
    private Vector2 _restrictedAreaBL;
    private Vector2 _wallJumpVector;
    private float _ignoreMovementFrames;
    private bool _frozen;
    private bool _originalKinematic;
    private float _timeScale = 1;
    private Vector3 _previousLoc;
    private Collider2D[] _collidersUpAgainst = new Collider2D[DIRECTIONS_CHECKED];
    private Vector2[] _collidedNormals = new Vector2[DIRECTIONS_CHECKED];
    private MotorState _prevState;
    private Bounds _prevColliderBounds;
    private float _dotAllowedForSlopes;
    private float _cornerDistanceCheck;
    private float _distanceFromEnvCorner;
    private Vector2 _bottomRight;
    private Vector3 _toTransform;
    private float _currentDeltaTime;
    private Rigidbody2D _rigidbody2D;
    private float _distanceToBoundsCorner;
    private float _savedTimeScale;
    private Vector2 _disallowedSlopeNormal;
    private Vector2 _previousMoveDir;
    private bool _isValidWallInteraction;

    // This is the unconverted motor velocity. This ignores slopes. It is converted into the appropriate vector before
    // moving.
    private Vector2 _velocity;

    // The function is cached to avoid unnecessary memory allocation.
    private EasingFunctions.EasingFunc _dashFunction;
    private EasingFunctions.EasingFunc _dashDerivativeFunction;

    // This is stored to notice if the public field changes during runtime.
    private EasingFunctions.Functions _currentDashEasingFunction;
    private float _currentWallJumpDegree;
    private float _currentSlopeDegreeAllowed;

    // Moving Platform Debug
    private Vector3 _point;
    private Vector3 _point2;
    private Bounds _prevPosPlat;
    private Bounds _startPosMotor;
    private Bounds _movedPosMotor;

    // Iteration Debug
    private int _iterationsUsed;
    private Bounds[] _iterationBounds;

    private Bounds _ladderArea;
    private Bounds _ladderBottomArea;
    private Bounds _ladderTopArea;

    // Contains the various jump variables, this is for organization.
    private class JumpState
    {
        public bool pressed;
        public bool held;
        public int numAirJumps;

        public int timeToldFrames;
        public int allowExtraFrames;

        public bool force;
        public float height;

        public float jumpGraceFrames;
        public bool jumpTypeChanged;

        public JumpType lastValidJump
        {
            get { return _lastValidJump; }
            set
            {
                if (value != JumpType.None)
                {
                    jumpTypeChanged = true;
                }
                else
                {
                    jumpGraceFrames = -1;
                }

                _lastValidJump = value;
            }
        }

        public enum JumpType
        {
            None,
            Normal,
            RightWall,
            LeftWall,
            Corner
        }

        private JumpType _lastValidJump;
    }
    private JumpState _jumping = new JumpState();

    private bool _ignoreGravity;

    // Contains the various dash variables.
    private class DashState
    {
        public bool pressed;
        public float cooldownFrames;
        public int dashingFrames;
        public bool dashWithDirection;
        public Vector2 dashDir = Vector2.zero;
        public float distanceCalculated;
        public float distanceDashed;
        public bool force;
        public float gravityEnabledFrames;
    }
    private DashState _dashing = new DashState();

    // Contains information for wall sticks, slides, and corner grabs.
    private class WallState
    {
        public float cornerHangFrames;
        public float stickFrames;

        public float wallInteractionCooldownFrames;
        public bool canHangAgain = true;
    }
    private WallState _wallInfo = new WallState();

    private class MovingPlatformState
    {
        public MovingPlatformMotor2D platform;

        public Vector2 previousPos;
        public CollidedSurface stuckToWall;
        public bool isOnPlatform { get { return platform != null; } }
    }
    private MovingPlatformState _movingPlatformState = new MovingPlatformState();

    // Used for environment checks and one way platforms
    private static RaycastHit2D[] _hits = new RaycastHit2D[STARTING_ARRAY_SIZE];
    private static RaycastHit2D[] _hitsNoDistance = new RaycastHit2D[STARTING_ARRAY_SIZE];

    private const float NEAR_ZERO = 0.0001f;

    private const float DISTANCE_TO_END_ITERATION = 0.001f;

    private const int STARTING_ARRAY_SIZE = 4;
    private const float INCREASE_ARRAY_SIZE_MULTIPLIER = 2;

    private const int DIRECTIONS_CHECKED = 4;
    private const int DIRECTION_DOWN = 0;
    private const int DIRECTION_UP = 1;
    private const int DIRECTION_LEFT = 2;
    private const int DIRECTION_RIGHT = 3;

    private Collider2D _collider2D { get; set; }

    private void Awake()
    {
        SetDashFunctions();
        _collider2D = GetComponent<Collider2D>();
        _rigidbody2D = GetComponent<Rigidbody2D>();

        if (iterationDebug)
        {
            _iterationBounds = new Bounds[2 + numOfIterations];
        }
    }

    private void Start()
    {
        _previousLoc = _collider2D.bounds.center;
        // initial set, do not use ChangeState
        motorState = MotorState.Falling;
        _wallJumpVector = Quaternion.AngleAxis(wallJumpAngle, Vector3.forward) * Vector3.right;
        _currentWallJumpDegree = wallJumpAngle;

        _cornerDistanceCheck = Mathf.Sqrt(2 * envCheckDistance * envCheckDistance);
        _distanceFromEnvCorner = Mathf.Sqrt(2 * minDistanceFromEnv * minDistanceFromEnv);

        _distanceToBoundsCorner = (_collider2D.bounds.max - _collider2D.bounds.center).magnitude;

        _bottomRight = new Vector2(1, -1).normalized;

        SetSlopeDegreeAllowed();

        ladderZone = LadderZone.Bottom;
        _collisionMask = staticEnvLayerMask | movingPlatformLayerMask;
    }

    private void OnEnable()
    {
        if (_rigidbody2D != null)
        {
            _velocity = _rigidbody2D.velocity;
            _originalKinematic = _rigidbody2D.isKinematic;
            _rigidbody2D.isKinematic = true;
        }
    }

    private void OnDisable()
    {
        if (_rigidbody2D != null)
        {
            _rigidbody2D.velocity = _velocity;
            _rigidbody2D.isKinematic = _originalKinematic;
        }
    }

    private void SetSlopeDegreeAllowed()
    {
        Vector3 v = Quaternion.AngleAxis(angleAllowedForMoving, Vector3.back) * Vector3.up;
        _dotAllowedForSlopes = Vector3.Dot(Vector3.up, v);
        _currentSlopeDegreeAllowed = angleAllowedForMoving;
    }

    private static Vector2 GetPointOnBounds(Bounds bounds, Vector3 toPoint)
    {
        // From http://stackoverflow.com/questions/4061576/finding-points-on-a-rectangle-at-a-given-angle
        float angle = Vector3.Angle(Vector3.right, toPoint);

        if (toPoint.y < 0)
        {
            angle = 360f - angle;
        }

        float multiplier = 1f;

        if ((angle >= 0f && angle < 45f) ||
            angle > 315f ||
            (angle >= 135f && angle < 225f))
        {

            if (angle >= 135f && angle < 225f)
            {
                multiplier = -1f;
            }

            return new Vector2(
                multiplier * bounds.size.x / 2 + bounds.center.x,
                bounds.center.y + multiplier * ((bounds.size.x / 2) * Mathf.Tan(angle * Mathf.Deg2Rad)));
        }

        if (angle >= 225f)
        {
            multiplier = -1f;
        }

        return new Vector2(
            bounds.center.x + multiplier * bounds.size.y / (2 * Mathf.Tan(angle * Mathf.Deg2Rad)),
            multiplier * bounds.size.y / 2 + bounds.center.y);
    }

    private void UpdateProperties()
    {
        if (angleAllowedForMoving != _currentSlopeDegreeAllowed)
        {
            SetSlopeDegreeAllowed();
        }

        if (_collider2D.bounds.size != _prevColliderBounds.size)
        {
            _distanceToBoundsCorner = _collider2D.bounds.extents.magnitude;
        }

        _toTransform = transform.position - _collider2D.bounds.center;
    }

    private void UpdateTimers()
    {
        // All timers in the motor are countdowns and are considered valid so long as the timer is >= 0
        _dashing.cooldownFrames--;
        _dashing.gravityEnabledFrames--;
        _dashing.dashingFrames--;
        _wallInfo.cornerHangFrames--;
        _wallInfo.stickFrames--;
        _wallInfo.wallInteractionCooldownFrames--;
        _ignoreMovementFrames--;
        _jumping.jumpGraceFrames--;
        _jumping.timeToldFrames--;
        _jumping.allowExtraFrames--;
    }

    private void ReadjustTimers(float multiplier)
    {
        _dashing.cooldownFrames = Mathf.RoundToInt(_dashing.cooldownFrames * multiplier);
        _dashing.gravityEnabledFrames = Mathf.RoundToInt(_dashing.gravityEnabledFrames * multiplier);
        _dashing.dashingFrames = Mathf.RoundToInt(_dashing.dashingFrames * multiplier);
        _wallInfo.cornerHangFrames = Mathf.RoundToInt(_wallInfo.cornerHangFrames * multiplier);
        _wallInfo.stickFrames = Mathf.RoundToInt(_wallInfo.stickFrames * multiplier);
        _wallInfo.wallInteractionCooldownFrames = Mathf.RoundToInt(_wallInfo.wallInteractionCooldownFrames * multiplier);
        _ignoreMovementFrames = Mathf.RoundToInt(_ignoreMovementFrames * multiplier);
        _jumping.jumpGraceFrames = Mathf.RoundToInt(_jumping.jumpGraceFrames * multiplier);
        _jumping.timeToldFrames = Mathf.RoundToInt(_jumping.timeToldFrames * multiplier);
        _jumping.allowExtraFrames = Mathf.RoundToInt(_jumping.allowExtraFrames * multiplier);
    }

    private void UpdateVelocity()
    {
        // First, are we trying to dash?
        if (enableDashes &&
            (_dashing.pressed &&
            _dashing.cooldownFrames < 0 &&
            !IsDashing() ||
            _dashing.force))
        {
            StartDash();
        }

        _dashing.pressed = false;

        if (!IsDashing())
        {
            // If we have standard control then facing can change any frame.
            SetFacing();

            ApplyMovement();

            if (IsForceSlipping())
            {
                Vector2 slopeDir = GetDownSlopeDir();

                if (fallFast)
                {
                    float increaseBy = -slopeDir.y * fastFallGravityMultiplier *
                        Mathf.Abs(Physics2D.gravity.y) * GetDeltaTime();

                    _velocity += slopeDir * increaseBy;

                    if (Vector2.Dot(GetMovementDir(_velocity.x), slopeDir) > NEAR_ZERO)
                    {
                        _velocity = Vector2.ClampMagnitude(_velocity, -slopeDir.y * fastFallSpeed);
                    }
                }
                else
                {
                    float increaseBy = -slopeDir.y * gravityMultiplier *
                        Mathf.Abs(Physics2D.gravity.y) * GetDeltaTime();

                    _velocity += slopeDir * increaseBy;

                    if (Vector2.Dot(GetMovementDir(_velocity.x), slopeDir) > NEAR_ZERO)
                    {
                        _velocity = Vector2.ClampMagnitude(_velocity, -slopeDir.y * fallSpeed);
                    }
                }
            }

            // Handle jumping.
            HandlePreJumping();

            // Any wall interactions.
            HandlePreWallInteraction();
        }
    }

    private float MoveMotor()
    {
        if (IsDashing())
        {
            float normalizedTime = (float)(GetFrameCount(dashDuration) - _dashing.dashingFrames) /
                GetFrameCount(dashDuration);

            if (_currentDashEasingFunction != dashEasingFunction)
            {
                // This allows the easing function to change during runtime and cut down on unnecessary allocations.
                SetDashFunctions();
            }

            float distance = _dashFunction(0, dashDistance, normalizedTime);

            _velocity = _dashing.dashDir * GetDashSpeed();
            MovePosition(_collider2D.bounds.center + (Vector3)_dashing.dashDir * (distance - _dashing.distanceCalculated));
            _dashing.distanceCalculated = distance;
            // Right now dash only moves along a line, doesn't ever need to adjust. We don't need multiple iterations for that.
            return 0;
        }

        Vector3 curPos = _collider2D.bounds.center;
        Vector3 targetPos = _collider2D.bounds.center + (Vector3)_velocity * GetDeltaTime();

        if (iterationDebug && _currentDeltaTime == Time.fixedDeltaTime)
        {
            _iterationBounds[0] = _collider2D.bounds;

            Bounds b = _collider2D.bounds;
            b.center = targetPos;

            _iterationBounds[1] = b;
        }

        MovePosition(_collider2D.bounds.center + (Vector3)_velocity * GetDeltaTime());

        if (numOfIterations == 1 ||
            (targetPos - _collider2D.bounds.center).sqrMagnitude < DISTANCE_TO_END_ITERATION * DISTANCE_TO_END_ITERATION)
        {
            return 0;
        }

        return Mathf.Lerp(_currentDeltaTime, 0,
            (_collider2D.bounds.center - curPos).magnitude / (targetPos - curPos).magnitude);
    }

    private void UpdateSurroundings(bool forceCheck)
    {
        bool currentOnSlope = onSlope;
        Vector2 currentSlopeNormal = slopeNormal;
        bool wasGrounded = IsGrounded();
        bool wasSlipping = IsForceSlipping();

        if (forceCheck)
        {
            collidingAgainst = CheckSurroundings(true);
        }
        else
        {
            // We may not need to check at all!
            if (velocity == Vector2.zero && normalizedXMovement == 0)
            {
                // No state change but if we are next to a slope we can't walk up then we should reset it to be handled.
                if (_disallowedSlopeNormal != Vector2.zero)
                {
                    // we update to the last disallowed slope normal then as well. This will be worked out.
                    onSlope = true;
                    slopeNormal = _disallowedSlopeNormal;
                }
            }
            else
            {
                collidingAgainst = CheckSurroundings(false);
            }
        }

        // If we are dashing then we can bail.
        if (IsDashing())
        {
            return;
        }

        // If we've moved on to a new slope (or to no slope from a slope) then we update our velocity vector to match
        if (wasGrounded &&
            velocity != Vector2.zero &&
            (HasFlag(CollidedSurface.Ground) || HasFlag(CollidedSurface.SlopeLeft) || HasFlag(CollidedSurface.SlopeRight)) &&
            !IsJumping() &&
            ((currentOnSlope && ((currentSlopeNormal != _disallowedSlopeNormal) || !onSlope)) ||
            currentOnSlope != onSlope))
        {

            // With our velocity pointing upwards, IsGrounded will return false. Since we want to stick to environments
            // this will fix our velocity.
            float speed;
            float maxSpeed;

            GetSpeedAndMaxSpeedOnGround(out speed, out maxSpeed);

            _velocity = GetMovementDir(velocity.x) * Mathf.Abs(speed);
        }

        _disallowedSlopeNormal = Vector2.zero;

        if (wasGrounded &&
            !wasSlipping &&
            IsForceSlipping())
        {
            Vector2 moveDir = _previousMoveDir;

            if (velocity != Vector2.zero)
            {
                moveDir = GetMovementDir(velocity.x);
                _previousMoveDir = moveDir;
            }

            // We weren't slipping but now we are, are we trying to go up a slope?
            if (Vector2.Dot(GetDownSlopeDir(), moveDir) < 0 &&
                velocity.sqrMagnitude < minimumSpeedToMoveUpSlipperySlope * minimumSpeedToMoveUpSlipperySlope)
            {
                // Don't allow.
                _velocity = Vector3.Project(velocity, slopeNormal);
                _disallowedSlopeNormal = slopeNormal;
                slopeNormal = currentSlopeNormal;
                onSlope = currentOnSlope;
            }
        }
    }

    private void UpdateState(bool forceSurroundingsCheck)
    {
        // Since this is in UpdateState, we can end dashing if the timer is at 0.
        if (IsDashing() && _dashing.dashingFrames <= 0)
        {
            EndDash();
        }

        UpdateSurroundings(forceSurroundingsCheck);

        if (IsDashing())
        {
            // Still dashing, nothing else matters.
            _dashing.distanceDashed += (_collider2D.bounds.center - _previousLoc).magnitude;
            return;
        }

        CheckWallInteractionValidity();

        if (HasFlag(CollidedSurface.Ground))
        {
            if (IsFalling() || IsFallingFast())
            {
                if (onLanded != null)
                {
                    onLanded();
                }
            }
        }

        AttachToMovingPlatforms();

        UpdateInformationFromMovement();

        HandlePostWallInteraction();

        // If our state is not in the air then open up the possibility of air jumps (we need to be able to air jump if
        // we walk off an edge so it can't be based of when a jump occurred).
        if (!IsInAir())
        {
            _jumping.numAirJumps = 0;
        }

        if (_velocity.y > 0 && HasFlag(CollidedSurface.Ceiling))
        {
            _velocity.y = 0;
        }

        if (IsGrounded())
        {
            Vector3 slopeDir = GetDownSlopeDir();

            if (IsForceSlipping() && _velocity != Vector2.zero && Mathf.Sign(_velocity.x) == Mathf.Sign(slopeDir.x))
            {
                float speed = _velocity.magnitude;

                _velocity = slopeDir * Vector2.Dot(_velocity / speed, slopeDir) * speed;
                ChangeState(MotorState.Slipping);
            }
            else
            {
                ChangeState(MotorState.OnGround);

                if (onSlope)
                {
                    _velocity = Vector3.Project(velocity, GetDownSlopeDir());
                }
                else
                {
                    _velocity.y = 0;
                }
            }
        }
        else if (IsOnGround() || IsSlipping())
        {
            ChangeState(MotorState.Falling);
        }

        if (_movingPlatformState.isOnPlatform)
        {
            _movingPlatformState.previousPos = _movingPlatformState.platform.position;
        }

        SetLastJumpType();
    }

    private void FixedUpdate()
    {
        // Frozen?
        if (frozen || timeScale == 0)
        {
            return;
        }

        UpdateTimers();

        // update _collisionMask in case it's updated by user
        _collisionMask = staticEnvLayerMask | movingPlatformLayerMask;

        float time = Time.fixedDeltaTime;
        int iterations = 0;
        _iterationsUsed = 0;

        while (time > 0 && iterations < numOfIterations)
        {
            time = UpdateMotor(time);
            iterations++;

            if (iterationDebug)
            {
                _iterationBounds[(_iterationsUsed++) + 2] = _collider2D.bounds;
            }
        }
        // check ladder zone
        if (_ladderBottomArea.Contains(_collider2D.bounds.center))
        {
            ladderZone = LadderZone.Bottom;
        }
        else if (_ladderTopArea.Contains(_collider2D.bounds.center))
        {
            ladderZone = LadderZone.Top;
        }
        else if (_ladderArea.Contains(_collider2D.bounds.center))
        {
            ladderZone = LadderZone.Middle;
        }
    }

    private float UpdateMotor(float deltaTime)
    {
        _currentDeltaTime = deltaTime;

        UpdateProperties();

        // The update phase is broken up into four phases each with certain responsibilities.

        // Phase One: Update internal state if something forced the motor into an unexpected state. This can be because
        //            a moving platform moved into us or our collider changed. This could means three UpdateStates, which is
        //            expensive, but we need to know our state.

        if (_prevColliderBounds != _collider2D.bounds)
        {
            UpdateState(true);
        }

        if (UpdateMovingPlatform())
        {
            UpdateState(true);
        }
        // Phase Two: Update internal representation of velocity
        UpdateVelocity();

        // Phase Three: Move the motor to the new location (and well update falling)
        deltaTime = MoveMotor();

        HandleFalling();

        // Phase Four: Update internal state so it can be accurately queried and ready for next step. We have to force all sides
        //             if there is moving platforms. They can move next to us, or away from us, without us knowing.
        UpdateState(movingPlatformLayerMask != 0);

        _prevColliderBounds = _collider2D.bounds;

        return deltaTime;
    }

    private bool UpdateMovingPlatform()
    {
        if (movingPlatformLayerMask == 0)
        {
            // No moving platforms, early bail.
            return false;
        }

        if (movingPlatformDebug)
        {
            _point = new Vector3();
            _point2 = new Vector3();
            _prevPosPlat = new Bounds();
            _movedPosMotor = new Bounds();
            _startPosMotor = new Bounds();
        }

        // Update location if on a moving platform.
        if (!IsDashing() && _movingPlatformState.isOnPlatform)
        {
            Vector3 toNewPos = _movingPlatformState.platform.position - _movingPlatformState.previousPos;
            transform.position += toNewPos;

            _prevColliderBounds = _collider2D.bounds;
        }

        Bounds checkBounds = _collider2D.bounds;
        checkBounds.extents += Vector3.one * minDistanceFromEnv;

        Collider2D col = Physics2D.OverlapArea(checkBounds.min, checkBounds.max, _collisionMask);

        if (col != null)
        {
            SeparateFromEnvirionment();
            return true;
        }

        return false;
    }

    private void SeparateFromEnvirionment()
    {
        // We'll start with the corners
        RaycastAndSeparate(
            (_collider2D.bounds.max - _collider2D.bounds.center) / _distanceToBoundsCorner,
            _distanceToBoundsCorner + _distanceFromEnvCorner);

        RaycastAndSeparate(
            (new Vector3(_collider2D.bounds.max.x, _collider2D.bounds.min.y) - _collider2D.bounds.center) /
            _distanceToBoundsCorner,
            _distanceToBoundsCorner + _distanceFromEnvCorner);

        RaycastAndSeparate(
            (_collider2D.bounds.min - _collider2D.bounds.center) / _distanceToBoundsCorner,
            _distanceToBoundsCorner + _distanceFromEnvCorner);

        RaycastAndSeparate(
            (new Vector3(_collider2D.bounds.min.x, _collider2D.bounds.max.y) - _collider2D.bounds.center) /
            _distanceToBoundsCorner,
            _distanceToBoundsCorner + _distanceFromEnvCorner);

        // These are more expensive...they only need to be done if piece of environment smaller (width/height) than the motor
        // can move into it. Otherwise just set additionalRaycastsPerSide to 0.

        // Top/Bottom
        for (int i = 0; i < additionalRaycastsPerSide; i++)
        {
            Vector2 dir = new Vector2(
                Mathf.Lerp(_collider2D.bounds.min.x, _collider2D.bounds.max.x, (float)(i + 1) / (additionalRaycastsPerSide + 1)),
                _collider2D.bounds.max.y) - (Vector2)_collider2D.bounds.center;

            float distance = dir.magnitude;

            RaycastAndSeparate(dir / distance, distance + minDistanceFromEnv);
            RaycastAndSeparate(-dir / distance, distance + minDistanceFromEnv);
        }

        // Right/Left
        for (int i = 0; i < additionalRaycastsPerSide; i++)
        {
            Vector2 dir = new Vector2(
                _collider2D.bounds.max.x,
                Mathf.Lerp(_collider2D.bounds.min.y, _collider2D.bounds.max.y,
                (float)(i + 1) / (additionalRaycastsPerSide + 1))) - (Vector2)_collider2D.bounds.center;

            float distance = dir.magnitude;

            RaycastAndSeparate(dir / distance, distance + minDistanceFromEnv);
            RaycastAndSeparate(-dir / distance, distance + minDistanceFromEnv);
        }
    }

    private void RaycastAndSeparate(Vector2 dir, float distance)
    {
        RaycastHit2D hit = GetClosestHit(_collider2D.bounds.center, dir, distance, false, true);

        if (hit.collider != null)
        {
            Vector2 pointOnCol = GetPointOnBounds(_collider2D.bounds, -hit.normal);

            Vector3 toPointOnCol = pointOnCol - hit.point;
            Vector3 pointToSepFrom = (Vector3)hit.point + Vector3.Project(toPointOnCol, -hit.normal);

            if (movingPlatformDebug)
            {
                _point = hit.point;
                _point2 = pointToSepFrom;
                _startPosMotor = _collider2D.bounds;
            }

            transform.position += ((Vector3)hit.point - pointToSepFrom) + (Vector3)hit.normal * minDistanceFromEnv;

            if (movingPlatformDebug)
            {
                _movedPosMotor = _collider2D.bounds;
            }
        }
    }

    private void CheckWallInteractionValidity()
    {
        _isValidWallInteraction = false;

        if (!enableWallSlides && !enableCornerGrabs && !enableWallSticks)
        {
            // Don't need the unnecessary check!
            return;
        }

        Vector2 min = _collider2D.bounds.min;
        Vector2 max = _collider2D.bounds.max;

        if (HasFlag(CollidedSurface.LeftWall) &&
            _collidedNormals[DIRECTION_LEFT] == Vector2.right &&
            normalizedXMovement < 0)
        {
            max.x = _collider2D.bounds.center.x;
            min.x = min.x - envCheckDistance;
        }
        else if (HasFlag(CollidedSurface.RightWall) &&
                 _collidedNormals[DIRECTION_RIGHT] == Vector2.left &&
                 normalizedXMovement > 0)
        {
            min.x = _collider2D.bounds.center.x;
            max.x = max.x + envCheckDistance;
        }
        else
        {
            return;
        }

        min.y = max.y - _collider2D.bounds.size.y * normalizedValidWallInteraction;

        _isValidWallInteraction = Physics2D.OverlapArea(min, max, _collisionMask) != null;
    }

    private void SetLastJumpType()
    {
        if (IsOnGround())
        {
            _jumping.lastValidJump = JumpState.JumpType.Normal;
        }
        else if (enableWallJumps)
        {
            if (PressingIntoLeftWall())
            {
                _jumping.lastValidJump = JumpState.JumpType.LeftWall;
            }
            else if (PressingIntoRightWall())
            {
                _jumping.lastValidJump = JumpState.JumpType.RightWall;
            }
        }
        else if (IsOnCorner())
        {
            _jumping.lastValidJump = JumpState.JumpType.Corner;
        }

        // We don't track air jumps as they are always valid in the air.
        if (_jumping.jumpTypeChanged && _jumping.lastValidJump != JumpState.JumpType.None)
        {
            _jumping.jumpTypeChanged = false;
            _jumping.jumpGraceFrames = GetFrameCount(jumpWindowWhenFalling);
        }


    }

    private void UpdateInformationFromMovement()
    {
        float diffInPositions = Mathf.Abs(_collider2D.bounds.center.y - _previousLoc.y);

        if (IsFalling() || IsFallingFast())
        {
            amountFallen += diffInPositions;

            if (IsFallingFast() && _velocity.y <= -fallSpeed)
            {
                amountFastFallen += diffInPositions;
            }
            else
            {
                amountFastFallen = 0;
            }
        }
        else
        {
            amountFallen = 0;
        }

        // Jumps
        if (IsJumping())
        {
            if (_velocity.y <= 0)
            {
                ChangeState(MotorState.Falling);
            }
            else
            {
                amountJumpedFor += diffInPositions;
            }
        }
    }

    private bool HasFlag(CollidedSurface cs)
    {
        return (collidingAgainst & cs) != CollidedSurface.None;
    }

    private float GetDeltaTime()
    {
        return _currentDeltaTime * timeScale;
    }

    private void SetDashFunctions()
    {
        _dashFunction = EasingFunctions.GetEasingFunction(dashEasingFunction);
        _dashDerivativeFunction = EasingFunctions.GetEasingFunctionDerivative(dashEasingFunction);
        _currentDashEasingFunction = dashEasingFunction;
    }

    private void AttachToMovingPlatforms()
    {
        if (movingPlatformLayerMask == 0)
        {
            // No platforms, no reason to do this
            return;
        }

        MovingPlatformMotor2D previous = _movingPlatformState.platform;
        _movingPlatformState.platform = null;
        _movingPlatformState.stuckToWall = CollidedSurface.None;

        if (HasFlag(CollidedSurface.Ground) && IsMovingPlatform(_collidersUpAgainst[DIRECTION_DOWN].gameObject))
        {
            _movingPlatformState.platform = _collidersUpAgainst[DIRECTION_DOWN].GetComponent<MovingPlatformMotor2D>();

            if (fallFast)
            {
                if (_movingPlatformState.platform.velocity.y < -fastFallSpeed)
                {
                    _movingPlatformState.platform = null;
                    _velocity.y = -fastFallSpeed;
                    collidingAgainst &= ~CollidedSurface.Ground;
                }
            }
            else
            {
                if (_movingPlatformState.platform.velocity.y < -fallSpeed)
                {
                    _movingPlatformState.platform = null;
                    _velocity.y = -fallSpeed;
                    collidingAgainst &= ~CollidedSurface.Ground;
                }
            }
        }
        else if ((IsOnWall() || IsGrounded()) &&
            PressingIntoLeftWall() &&
            IsMovingPlatform(_collidersUpAgainst[DIRECTION_LEFT].gameObject))
        {
            // We allow the motor to attach when pressing into a moving platform. This prevent jitter as it's moving away from
            // the motor.
            _movingPlatformState.platform = _collidersUpAgainst[DIRECTION_LEFT].GetComponent<MovingPlatformMotor2D>();
            _movingPlatformState.stuckToWall = CollidedSurface.LeftWall;
        }
        else if ((IsOnWall() || IsGrounded()) &&
            PressingIntoRightWall() &&
            IsMovingPlatform(_collidersUpAgainst[DIRECTION_RIGHT].gameObject))
        {
            _movingPlatformState.platform = _collidersUpAgainst[DIRECTION_RIGHT].GetComponent<MovingPlatformMotor2D>();
            _movingPlatformState.stuckToWall = CollidedSurface.RightWall;
        }

        if (_movingPlatformState.platform != null && _movingPlatformState.platform != previous)
        {
            if (_movingPlatformState.platform.onPlatformerMotorContact != null)
            {
                _movingPlatformState.platform.onPlatformerMotorContact.Invoke(this);
            }
        }
    }

    private void HandlePreJumping()
    {
        if (_jumping.timeToldFrames >= 0)
        {
            _jumping.pressed = true;
        }

        if (_currentWallJumpDegree != wallJumpAngle)
        {
            _wallJumpVector = Quaternion.AngleAxis(wallJumpAngle, Vector3.forward) * Vector3.right;
            _currentWallJumpDegree = wallJumpAngle;
        }

        // If we're currently jumping and the jump button is still held down ignore gravity to allow us to achieve the extra
        // height.
        if (IsJumping() && _jumping.held && _jumping.allowExtraFrames > 0)
        {
            _ignoreGravity = true;
        }
        else if (_ignoreGravity)
        {
            _ignoreGravity = false;
        }

        // Jump?
        if (_jumping.pressed)
        {
            bool jumped = true;

            // Jump might mean different things depending on the state.
            if ((_jumping.lastValidJump == JumpState.JumpType.Normal && _jumping.jumpGraceFrames >= 0) ||
                IsOnGround() ||
                IsSlipping() ||
                _jumping.force)
            {
                // Normal jump.
                if (IsForceSlipping())
                {
                    _velocity = slopeNormal * CalculateSpeedNeeded(_jumping.height);
                }
                else
                {
                    _velocity.y = CalculateSpeedNeeded(_jumping.height);
                }
            }
            else if (IsOnCorner() ||
                     _jumping.lastValidJump == JumpState.JumpType.Corner && _jumping.jumpGraceFrames >= 0)
            {
                // If we are on a corner then jump up.
                _velocity = Vector2.up * CalculateSpeedNeeded(_jumping.height) * cornerJumpMultiplier;
                _ignoreMovementFrames = GetFrameCount(ignoreMovementAfterJump);

                if (onCornerJump != null)
                {
                    onCornerJump();
                }
            }
            else if (enableWallJumps &&
                ((_jumping.lastValidJump == JumpState.JumpType.LeftWall && _jumping.jumpGraceFrames >= 0) ||
                (_isValidWallInteraction && PressingIntoLeftWall())))
            {
                // If jump was pressed as we or before we entered the wall then just jump away.
                _velocity = _wallJumpVector * CalculateSpeedNeeded(_jumping.height) * wallJumpMultiplier;

                // It's likely the player is still pressing into the wall, ignore movement for a little amount of time.
                // TODO: Only ignore left movement?
                _ignoreMovementFrames = GetFrameCount(ignoreMovementAfterJump);

                // If wall jump is allowed but not wall slide then double jump will not be allowed earlier, allow it now.
                _jumping.numAirJumps = 0;

                if (onWallJump != null)
                {
                    onWallJump(Vector2.right);
                }
            }
            else if (enableWallJumps &&
                ((_jumping.lastValidJump == JumpState.JumpType.RightWall && _jumping.jumpGraceFrames >= 0) ||
                (_isValidWallInteraction && PressingIntoRightWall())))
            {

                _velocity = _wallJumpVector * CalculateSpeedNeeded(_jumping.height) * wallJumpMultiplier;
                _velocity.x *= -1;

                _ignoreMovementFrames = GetFrameCount(ignoreMovementAfterJump);
                _jumping.numAirJumps = 0;

                if (onWallJump != null)
                {
                    onWallJump(Vector2.left);
                }
            }
            else if (_jumping.numAirJumps < numOfAirJumps)
            {
                _velocity.y = CalculateSpeedNeeded(_jumping.height);
                _jumping.numAirJumps++;

                if (onAirJump != null)
                {
                    onAirJump();
                }
            }
            else
            {
                // Guess we aren't jumping!
                jumped = false;
            }

            if (jumped)
            {
                _jumping.pressed = false;
                _jumping.force = false;
                _jumping.allowExtraFrames = GetFrameCount(extraJumpHeight / CalculateSpeedNeeded(_jumping.height));
                amountJumpedFor = 0;
                _movingPlatformState.platform = null;
                _jumping.lastValidJump = JumpState.JumpType.None;
                _jumping.timeToldFrames = -1;

                ChangeState(MotorState.Jumping);
            }
        }

        _jumping.pressed = false;
    }

    private bool PressingIntoLeftWall()
    {
        if (_movingPlatformState.isOnPlatform &&
            _movingPlatformState.stuckToWall == CollidedSurface.LeftWall &&
            normalizedXMovement < -wallInteractionThreshold)
        {
            return true;
        }

        return HasFlag(CollidedSurface.LeftWall) && normalizedXMovement < -wallInteractionThreshold;
    }

    private bool PressingIntoRightWall()
    {
        if (_movingPlatformState.isOnPlatform &&
            _movingPlatformState.stuckToWall == CollidedSurface.RightWall &&
            normalizedXMovement > wallInteractionThreshold)
        {
            return true;
        }

        return HasFlag(CollidedSurface.RightWall) && normalizedXMovement > wallInteractionThreshold;
    }

    private void HandlePostWallInteraction()
    {
        // We can grab corners or walls again.
        if (!PressingIntoLeftWall() && !PressingIntoRightWall())
        {
            if (!_wallInfo.canHangAgain)
            {
                // Debounce time, mostly for platforms that are moving down and constantly triggering sticks
                _wallInfo.wallInteractionCooldownFrames = GetFrameCount(wallInteractionCooldown);
            }

            _wallInfo.canHangAgain = true;

            if (IsOnCorner() ||
                IsWallSticking() ||
                IsWallSliding())
            {
                ChangeState(MotorState.Falling);
            }
        }
        else
        {
            if (IsOnCorner())
            {
                if (!_isValidWallInteraction || !CheckIfAtCorner())
                {
                    ChangeState(MotorState.Falling);
                }
                else if (_wallInfo.cornerHangFrames < 0)
                {
                    ChangeState(enableWallSlides ? MotorState.WallSliding : MotorState.Falling);
                }
            }

            if (IsWallSticking())
            {
                if (!_isValidWallInteraction || !(PressingIntoLeftWall() || PressingIntoRightWall()))
                {
                    ChangeState(MotorState.Falling);
                }
                else if (_wallInfo.stickFrames < 0)
                {
                    ChangeState(enableWallSlides ? MotorState.WallSliding : MotorState.Falling);
                }
            }
        }


        if (HasFlag(CollidedSurface.Ground))
        {
            if (IsWallSliding() || _ignoreGravity)
            {
                _velocity.y = 0;
            }

            ChangeState(MotorState.OnGround);
        }
    }

    /// <summary>
    /// HandlePreWalInteraction is responsible for saying we are now on a wall and setting velocity to 0.
    /// </summary>
    private void HandlePreWallInteraction()
    {
        if (IsJumping() || _wallInfo.wallInteractionCooldownFrames >= 0 || HasFlag(CollidedSurface.Ground))
        {
            return;
        }

        // Corner grab?
        if (enableCornerGrabs)
        {
            if (_isValidWallInteraction && _velocity.y <= 0 && CheckIfAtCorner() && _wallInfo.canHangAgain)
            {
                _wallInfo.cornerHangFrames = GetFrameCount(cornerGrabDuration);
                _wallInfo.canHangAgain = false;
                _velocity = Vector2.zero;
                ChangeState(MotorState.OnCorner);
                return;
            }
        }

        // Wall Sticks
        if (enableWallSticks)
        {
            if (_isValidWallInteraction && _velocity.y <= 0 && (PressingIntoLeftWall() || PressingIntoRightWall()) && _wallInfo.canHangAgain)
            {
                _wallInfo.stickFrames = GetFrameCount(wallSticksDuration);
                _velocity = Vector2.zero;
                _wallInfo.canHangAgain = false;
                ChangeState(MotorState.WallSticking);
                return;
            }
        }

        // Wall slide?
        if (_isValidWallInteraction &&
            enableWallSlides &&
            !IsWallSticking() &&
            !IsOnCorner())
        {
            if (_velocity.y <= 0 && (PressingIntoLeftWall() || PressingIntoRightWall()) && !IsGrounded())
            {
                ChangeState(MotorState.WallSliding);
            }
        }

        if (IsWallSliding())
        {
            if (_velocity.y != -wallSlideSpeed)
            {
                if (timeToWallSlideSpeed != 0)
                {
                    if (_velocity.y > -wallSlideSpeed)
                    {
                        _velocity = Vector2.up * Accelerate(_velocity.y, -wallSlideSpeed / timeToWallSlideSpeed, -wallSlideSpeed);
                    }
                    else
                    {
                        _velocity = Vector2.up * Decelerate(
                            _velocity.y,
                            Mathf.Abs(wallSlideSpeed / timeToWallSlideSpeed),
                            -wallSlideSpeed);
                    }
                }
                else
                {
                    _velocity = Vector2.down * wallSlideSpeed;
                }
            }
        }
    }

    private void HandleFalling()
    {
        if (_ignoreGravity)
        {
            return;
        }

        if (IsInAir() && !_ignoreGravity)
        {
            // If we are falling fast then multiply the gravityMultiplier.
            if (fallFast)
            {
                if (_velocity.y == -fastFallSpeed)
                {
                    return;
                }

                if (_velocity.y > -fastFallSpeed)
                {
                    _velocity.y = Accelerate(
                        _velocity.y,
                        fastFallGravityMultiplier * Physics2D.gravity.y,
                        -fastFallSpeed);
                }
                else
                {
                    _velocity.y = Decelerate(
                        _velocity.y,
                        Mathf.Abs(fastFallGravityMultiplier * Physics.gravity.y),
                        -fastFallSpeed);
                }

                if (_velocity.y <= 0)
                {
                    ChangeState(MotorState.FallingFast);
                }
            }
            else
            {
                if (_dashing.gravityEnabledFrames < 0)
                {
                    if (_velocity.y == -fallSpeed)
                    {
                        return;
                    }

                    if (_velocity.y > -fallSpeed)
                    {
                        _velocity.y = Accelerate(
                            _velocity.y,
                            gravityMultiplier * Physics2D.gravity.y,
                            -fallSpeed);
                    }
                    else
                    {
                        _velocity.y = Decelerate(
                            _velocity.y,
                            Mathf.Abs(gravityMultiplier * Physics.gravity.y),
                            -fallSpeed);
                    }
                }

                if (_velocity.y <= 0)
                {
                    ChangeState(MotorState.Falling);
                }
            }
        }
    }

    private void ApplyMovement()
    {
        // Apply movement only if we're not ignoring it.
        if (_ignoreMovementFrames >= 0)
        {
            return;
        }

        float speed;
        float maxSpeed;

        // ladder check
        if (IsOnLadder() && IsUserHandled())
        {
            _velocity.x = normalizedXMovement * ladderSpeed;
            _velocity.y = normalizedYMovement * ladderSpeed;
        }
        // PERF: Optimal math out the window in favor of ease of figuring out, can resolve later if a problem.
        else if (Mathf.Abs(normalizedXMovement) > 0)
        {
            if (IsGrounded())
            {
                Vector2 moveDir = GetMovementDir(normalizedXMovement);

                if (IsForceSlipping() && Vector2.Dot(moveDir, GetDownSlopeDir()) <= NEAR_ZERO)
                {
                    // Don't allow walking up a slope that we slide down.
                    _velocity = GetMovementDir(_velocity.x) * _velocity.magnitude;
                    return;
                }

                GetSpeedAndMaxSpeedOnGround(out speed, out maxSpeed);

                if (timeToGroundSpeed > 0)
                {
                    // If we're moving faster than our normalizedXMovement * groundSpeed then decelerate rather than
                    // accelerate.
                    //
                    // Or if we are trying to move in the direction opposite of where we are facing.

                    if (speed > 0 &&
                        normalizedXMovement > 0 &&
                        speed > normalizedXMovement * maxSpeed ||
                        speed < 0 &&
                        normalizedXMovement < 0 &&
                        speed < normalizedXMovement * maxSpeed ||
                        speed < 0 &&
                        normalizedXMovement > 0 ||
                        speed > 0 &&
                        normalizedXMovement < 0)
                    {
                        float deceleration = (maxSpeed * maxSpeed) / (2 * groundStopDistance);

                        if (onSlope && changeSpeedOnSlopes)
                        {
                            float factor = (speedMultiplierOnSlope * (1 - slopeNormal.y));

                            if (moveDir.y > 0)
                            {
                                deceleration /= factor;
                            }
                            else
                            {
                                deceleration *= factor;
                            }
                        }

                        speed = Decelerate(
                            speed,
                            deceleration,
                            normalizedXMovement * maxSpeed);
                    }
                    else
                    {
                        float acceleration = normalizedXMovement * (maxSpeed / timeToGroundSpeed);

                        if (onSlope && changeSpeedOnSlopes)
                        {
                            float factor = (speedMultiplierOnSlope * (1 - slopeNormal.y));

                            if (moveDir.y < 0)
                            {
                                acceleration /= factor;
                            }
                            else
                            {
                                acceleration *= factor;
                            }
                        }

                        speed = Accelerate(
                            speed,
                            acceleration,
                            normalizedXMovement * maxSpeed);
                    }
                }
                else
                {
                    // We can overwrite y if we're on the ground, it's fine.
                    speed = normalizedXMovement * maxSpeed;
                }

                _velocity = GetMovementDir(speed) * Mathf.Abs(speed);
            }
            else if (changeDirectionInAir)
            {
                // Air doesn't have to change how it represents speed.
                if (timeToAirSpeed > 0)
                {
                    if (_velocity.x > 0 &&
                        normalizedXMovement > 0 &&
                        _velocity.x > normalizedXMovement * airSpeed ||
                        _velocity.x < 0 &&
                        normalizedXMovement < 0 &&
                        _velocity.x < normalizedXMovement * airSpeed)
                    {
                        speed = Decelerate(
                            _velocity.x,
                            (airSpeed * airSpeed) / (2 * airStopDistance),
                            normalizedXMovement * airSpeed);
                    }
                    else
                    {
                        speed = Accelerate(
                            _velocity.x,
                            normalizedXMovement * (airSpeed / timeToAirSpeed),
                            normalizedXMovement * airSpeed);
                    }
                }
                else
                {
                    speed = normalizedXMovement * airSpeed;
                }

                _velocity.x = speed;
            }
        }
        else if (_velocity.x != 0)
        {
            if (IsGrounded())
            {
                if (!IsSlipping())
                {
                    GetSpeedAndMaxSpeedOnGround(out speed, out maxSpeed);

                    if (groundStopDistance > 0)
                    {
                        float deceleration = (groundSpeed * groundSpeed) / (2 * groundStopDistance);

                        if (onSlope && changeSpeedOnSlopes)
                        {
                            float factor = (speedMultiplierOnSlope * (1 - slopeNormal.y));

                            if (GetMovementDir(_velocity.x).y > 0)
                            {
                                deceleration /= factor;
                            }
                            else
                            {
                                deceleration *= factor;
                            }
                        }

                        speed = Decelerate(speed, deceleration, 0);
                    }
                    else
                    {
                        speed = 0;
                    }

                    _velocity = GetMovementDir(speed) * Mathf.Abs(speed);
                }
            }
            else
            {
                if (airStopDistance > 0)
                {
                    speed = Decelerate(_velocity.x, (airSpeed * airSpeed) / (2 * airStopDistance), 0);
                }
                else
                {
                    speed = 0;
                }

                _velocity.x = speed;
            }
        }

        // These mean we can't progress forward. Either a wall or a slope
        if (HasFlag(CollidedSurface.LeftWall) &&
            _velocity.x < 0 &&
            _collidedNormals[DIRECTION_LEFT] == Vector2.right ||
            HasFlag(CollidedSurface.RightWall) &&
            _velocity.x > 0 &&
            _collidedNormals[DIRECTION_RIGHT] == Vector2.left)
        {
            _velocity.x = 0;
        }

        if (IsGrounded() &&
            _disallowedSlopeNormal != Vector2.zero &&
            (_disallowedSlopeNormal.x < 0 && _velocity.x > 0 ||
                _disallowedSlopeNormal.x > 0 && _velocity.x < 0) &&
            velocity.sqrMagnitude < minimumSpeedToMoveUpSlipperySlope * minimumSpeedToMoveUpSlipperySlope)
        {
            velocity = Vector2.zero;
        }
    }

    private void GetSpeedAndMaxSpeedOnGround(out float speed, out float maxSpeed)
    {
        Vector3 moveDir = GetMovementDir(_velocity.x);

        if (onSlope)
        {
            speed = velocity.magnitude * Mathf.Sign(_velocity.x);
            Vector3 slopeDir = GetDownSlopeDir();

            if (IsForceSlipping() && Vector2.Dot(moveDir, slopeDir) > NEAR_ZERO)
            {
                if (!changeSpeedOnSlopes)
                {
                    maxSpeed = fallFast ? fastFallSpeed : fallSpeed;
                }
                else
                {
                    maxSpeed = -slopeDir.y * (fallFast ? fastFallSpeed : fallSpeed);
                }
            }

            else if (changeSpeedOnSlopes)
            {
                if (moveDir.y > 0)
                {
                    maxSpeed = groundSpeed *
                        Vector3.Dot(Vector3.right * Mathf.Sign(moveDir.x), moveDir) *
                        speedMultiplierOnSlope;
                }
                else
                {
                    maxSpeed = groundSpeed *
                        (2f - Vector3.Dot(Vector3.right * Mathf.Sign(moveDir.x), moveDir) *
                        speedMultiplierOnSlope);
                }
            }
            else
            {
                maxSpeed = groundSpeed;
            }
        }
        else
        {
            speed = _velocity.x;
            maxSpeed = groundSpeed;
        }
    }

    private float Accelerate(float speed, float acceleration, float limit)
    {
        // acceleration can be negative or positive to note acceleration in that direction.
        speed += acceleration * GetDeltaTime();

        if (acceleration > 0)
        {
            if (speed > limit)
            {
                speed = limit;
            }
        }
        else
        {
            if (speed < limit)
            {
                speed = limit;
            }
        }

        return speed;
    }

    private float Decelerate(float speed, float deceleration, float limit)
    {
        // deceleration is always positive but assumed to take the velocity backwards.
        if (speed < 0)
        {
            speed += deceleration * GetDeltaTime();

            if (speed > limit)
            {
                speed = limit;
            }
        }
        else if (speed > 0)
        {
            speed -= deceleration * GetDeltaTime();

            if (speed < limit)
            {
                speed = limit;
            }
        }

        return speed;
    }

    private void StartDash()
    {
        // Set facing now and it won't be set again during dash.
        SetFacing();

        if (!_dashing.dashWithDirection)
        {
            // We dash depending on our direction.
            _dashing.dashDir = facingLeft ? Vector2.left : Vector2.right;
        }

        _dashing.distanceDashed = 0;
        _dashing.distanceCalculated = 0;
        _previousLoc = _collider2D.bounds.center;

        // This will begin the dash this frame.
        _dashing.dashingFrames = GetFrameCount(dashDuration) - 1;
        _dashing.force = false;

        ChangeState(MotorState.Dashing);
    }

    private float GetDashSpeed()
    {
        float normalizedTime = (float)(GetFrameCount(dashDuration) - _dashing.dashingFrames) /
            GetFrameCount(dashDuration);

        float speed = _dashDerivativeFunction(0, dashDistance, normalizedTime) / dashDuration;

        // Some of the easing functions may result in infinity, we'll uh, lower our expectations and make it maxfloat.
        // This will almost certainly be clamped.
        if (float.IsNegativeInfinity(speed))
        {
            speed = float.MinValue;
        }
        else if (float.IsPositiveInfinity(speed))
        {
            speed = float.MaxValue;
        }

        return speed;
    }

    private void MovePosition(Vector3 newPos)
    {
        if (newPos == _collider2D.bounds.center)
        {
            _previousLoc = _collider2D.bounds.center;
            return;
        }

        Vector3 toNewPos = newPos - _collider2D.bounds.center;
        float distance = toNewPos.magnitude;

        RaycastHit2D hit = GetClosestHit(_collider2D.bounds.center, toNewPos / distance, distance);

        //Debug.Log(enableSlopes + " " + _collider2D.bounds.center.x + " " + _collider2D.bounds.center.y + " " + (toNewPos / distance).x + " " + (toNewPos / distance).y + " " + distance);
        //Debug.Log(hit.collider + " " + hit.centroid.x + " " + hit.centroid.y + " " + hit.normal.x + " " + hit.normal.y);

        _previousLoc = _collider2D.bounds.center;

        if (hit.collider != null)
        {
            transform.position = _toTransform + (Vector3)hit.centroid + (Vector3)hit.normal * minDistanceFromEnv;
        }
        else
        {
            transform.position = _toTransform + newPos;
        }

        // at the end if there is a restricted area, force the motor inside
        // TODO handle rotation, unrotate transform.position, check, rotate
        if (IsRestricted())
        {
            Vector2 pos;
            pos.x = Mathf.Clamp(transform.position.x, _restrictedAreaBL.x, _restrictedAreaTR.x);
            pos.y = Mathf.Clamp(transform.position.y, _restrictedAreaBL.y, _restrictedAreaTR.y);
            transform.position = pos;
        }
    }

    private Vector3 GetMovementDir(float speed)
    {
        Vector3 moveDir = Vector3.zero;

        float multiplier = Mathf.Sign(speed);

        if (speed == 0)
        {
            multiplier = Mathf.Sign(normalizedXMovement);
            speed = normalizedXMovement;
        }

        if (!onSlope &&
            ((!HasFlag(CollidedSurface.SlopeLeft) && !HasFlag(CollidedSurface.SlopeRight)) ||
            (facingLeft && HasFlag(CollidedSurface.SlopeRight)) ||
            (!facingLeft && HasFlag(CollidedSurface.SlopeLeft)) ||
            (Vector3.Dot(Vector3.up, slopeNormal) < _dotAllowedForSlopes)))
        {
            return Mathf.Sign(speed) * Vector3.right;
        }

        moveDir.x = multiplier * slopeNormal.y;

        if (slopeNormal.x * speed > 0)
        {
            moveDir.y = -Mathf.Abs(slopeNormal.x);
        }
        else
        {
            moveDir.y = Mathf.Abs(slopeNormal.x);
        }

        return moveDir;
    }

    private Vector3 GetDownSlopeDir()
    {
        if (!onSlope)
        {
            return Vector3.zero;
        }

        Vector3 downDir = Vector3.zero;
        downDir.x = Mathf.Sign(slopeNormal.x) * slopeNormal.y;
        downDir.y = -Math.Abs(slopeNormal.x);

        return downDir;
    }

    private void SetFacing()
    {
        if (normalizedXMovement < 0)
        {
            facingLeft = true;
        }
        else if (normalizedXMovement > 0)
        {
            facingLeft = false;
        }
    }

    private bool CheckIfAtCorner()
    {
        Bounds box = _collider2D.bounds;

        Vector2 min = box.min;
        Vector2 max = box.max;

        // New min y is always at the current max y.
        min.y = max.y;
        max.y += cornerDistanceCheck;

        if (PressingIntoLeftWall())
        {
            max.x = min.x;
            min.x -= cornerDistanceCheck;
        }
        else if (PressingIntoRightWall())
        {
            min.x = max.x;
            max.x += cornerDistanceCheck;
        }
        else
        {
            return false;
        }

        Collider2D col = Physics2D.OverlapArea(min, max, _collisionMask);

        return (col == null);
    }

    private int GetNearbyHitsBox(
        Vector2 direction,
        float distance,
        bool useExternalHits)
    {
        int num = Physics2D.BoxCastNonAlloc(
            _collider2D.bounds.center,
            _collider2D.bounds.size,
            0f,
            direction,
            useExternalHits ? _hits : _hitsNoDistance,
            distance,
            _collisionMask);

        if (num > _hits.Length)
        {
            if (useExternalHits)
            {
                _hits = new RaycastHit2D[(int)(INCREASE_ARRAY_SIZE_MULTIPLIER * _hits.Length)];
            }
            else
            {
                _hitsNoDistance = new RaycastHit2D[(int)(INCREASE_ARRAY_SIZE_MULTIPLIER * _hitsNoDistance.Length)];
            }
        }

        num = Physics2D.BoxCastNonAlloc(
            _collider2D.bounds.center,
            _collider2D.bounds.size,
            0f,
            direction,
            useExternalHits ? _hits : _hitsNoDistance,
            distance,
            _collisionMask);

        return num;
    }

    private int GetNearbyHitsRay(
        Vector2 origin,
        Vector2 direction,
        float distance,
        bool useExternalHits)
    {
        int num = Physics2D.RaycastNonAlloc(
            origin,
            direction,
            useExternalHits ? _hits : _hitsNoDistance,
            distance,
            _collisionMask);

        if (num > _hits.Length)
        {
            if (useExternalHits)
            {
                _hits = new RaycastHit2D[(int)(INCREASE_ARRAY_SIZE_MULTIPLIER * _hits.Length)];
            }
            else
            {
                _hitsNoDistance = new RaycastHit2D[(int)(INCREASE_ARRAY_SIZE_MULTIPLIER * _hitsNoDistance.Length)];
            }
        }

        return Physics2D.RaycastNonAlloc(
            origin,
            direction,
            useExternalHits ? _hits : _hitsNoDistance,
            distance,
            _collisionMask);
    }

    private RaycastHit2D GetClosestHit(
        Vector2 origin,
        Vector3 direction,
        float distance,
        bool useBox = true,
        bool checkWereTouching = false)
    {
        if (!enableOneWayPlatforms && oneWayPlatformsAreWalls)
        {
            // This is much easier if we don't care about one way platforms.
            if (useBox)
            {
                return Physics2D.BoxCast(
                    origin,
                    _collider2D.bounds.size,
                    0f,
                    direction,
                    distance,
                    _collisionMask);
            }

            return Physics2D.Raycast(origin, direction, distance, _collisionMask);
        }

        // For one way platforms, things get interesting!

        int numOfHits;

        if (useBox)
        {
            numOfHits = GetNearbyHitsBox(
                direction,
                distance,
                true);
        }
        else
        {
            numOfHits = GetNearbyHitsRay(
                origin,
                direction,
                distance,
                true);
        }

        RaycastHit2D closestHit = new RaycastHit2D();
        float closeBy = float.MaxValue;

        for (int i = 0; i < numOfHits; i++)
        {
            if (_hits[i].collider.usedByEffector &&
                _hits[i].collider.GetComponent<PlatformEffector2D>().useOneWay)
            {
                // ignore OWP ?
                if (!oneWayPlatformsAreWalls) continue;

                bool isTouching = false;

                // You'd think OverlapArea would be sufficient but doesn't
                // appear to necessarily reliably return the expected colliders
                // So we box cast a distance of 0 instead.
                int numOfNoDistanceHits = GetNearbyHitsBox(
                    direction,
                    0f,
                    false);

                for (int j = 0; j < numOfNoDistanceHits; j++)
                {
                    if (_hitsNoDistance[j].collider == _hits[i].collider)
                    {
                        isTouching = true;
                        break;
                    }
                }

                if (isTouching)
                {
                    if (checkWereTouching && ((1 << _hits[i].collider.gameObject.layer) & movingPlatformLayerMask) != 0)
                    {
                        // If it's a moving platform then we need to know if we were touching.
                        MovingPlatformMotor2D mpMotor = _hits[i].collider.GetComponent<MovingPlatformMotor2D>();
                        Vector3 curPos = mpMotor.transform.position;
                        mpMotor.transform.position = mpMotor.previousPosition;
                        bool wasTouching = false;

                        numOfNoDistanceHits = GetNearbyHitsBox(
                            direction,
                            0f,
                            false);

                        mpMotor.transform.position = curPos;

                        for (int j = 0; j < numOfNoDistanceHits; j++)
                        {
                            if (_hitsNoDistance[j].collider == _hits[i].collider)
                            {
                                wasTouching = true;
                                break;
                            }
                        }

                        if (wasTouching)
                        {
                            continue;
                        }

                    }
                    else
                    {
                        continue;
                    }
                }

                Vector3 oneWayPlatformForward = _hits[i].collider.transform.TransformDirection(Vector3.up);
                float dot = 0;

                if (_velocity != Vector2.zero)
                {
                    dot = Vector3.Dot(
                        oneWayPlatformForward,
                        _velocity);
                }
                else if (((1 << _hits[i].collider.gameObject.layer) & movingPlatformLayerMask) != 0)
                {
                    // If we aren't moving then it's interesting if it's a moving platform.
                    MovingPlatformMotor2D mpMotor = _hits[i].collider.GetComponent<MovingPlatformMotor2D>();

                    if (_movingPlatformState.platform != mpMotor)
                    {
                        // This might break for more complicated one way platform moving platforms but it'll have to do.
                        if (mpMotor.velocity != Vector2.zero)
                        {
                            dot = Vector3.Dot(oneWayPlatformForward, -velocity);
                        }
                        else
                        {
                            Vector2 toNewPlatformPos = mpMotor.position - mpMotor.previousPosition;
                            dot = Vector3.Dot(oneWayPlatformForward, -toNewPlatformPos);
                        }
                    }
                }

                if (dot > NEAR_ZERO)
                {
                    continue;
                }
            }

            Vector3 toHit = _collider2D.bounds.center - (Vector3)_hits[i].centroid;

            if (toHit.sqrMagnitude < closeBy)
            {
                closeBy = toHit.sqrMagnitude;
                closestHit = _hits[i];
            }
        }

        return closestHit;
    }

    private CollidedSurface CheckGround(float distance, bool forceDistance = false)
    {
        CollidedSurface surfaces = CollidedSurface.None;

        RaycastHit2D closestHit = GetClosestHit(_collider2D.bounds.center, Vector3.down, distance);

        _collidersUpAgainst[DIRECTION_DOWN] = closestHit.collider;
        _collidedNormals[DIRECTION_DOWN] = closestHit.normal;

        if (closestHit.collider != null)
        {
            surfaces |= CollidedSurface.Ground;

            if (IsUserHandled())
            {
                FreedomStateExit();
                DisableRestrictedArea();
            }

            if (_collider2D.bounds.center.y - closestHit.centroid.y < minDistanceFromEnv || forceDistance)
            {
                transform.position += (minDistanceFromEnv -
                    (_collider2D.bounds.center.y - closestHit.centroid.y)) * Vector3.up;
            }
        }

        return surfaces;
    }

    private CollidedSurface CheckSurroundings(bool forceCheck)
    {
        CollidedSurface surfaces = CollidedSurface.None;

        Vector2 vecToCheck = _velocity;

        if (!forceCheck)
        {
            if (vecToCheck == Vector2.zero)
            {
                vecToCheck = Vector3.right * normalizedXMovement;
            }
        }

        RaycastHit2D closestHit;

        // Left
        if (forceCheck || -vecToCheck.x >= -NEAR_ZERO)
        {
            closestHit = GetClosestHit(_collider2D.bounds.center, Vector3.left, envCheckDistance);

            _collidersUpAgainst[DIRECTION_LEFT] = closestHit.collider;
            _collidedNormals[DIRECTION_LEFT] = closestHit.normal;

            if (closestHit.collider != null)
            {
                surfaces |= CollidedSurface.LeftWall;

                if (_collider2D.bounds.center.x - closestHit.centroid.x < minDistanceFromEnv)
                {
                    transform.position += (minDistanceFromEnv -
                        (_collider2D.bounds.center.x - closestHit.centroid.x)) * Vector3.right;
                }
            }

        }

        // Ceiling
        if (forceCheck || vecToCheck.y >= -NEAR_ZERO)
        {
            closestHit = GetClosestHit(_collider2D.bounds.center, Vector3.up, envCheckDistance);

            _collidersUpAgainst[DIRECTION_UP] = closestHit.collider;
            _collidedNormals[DIRECTION_UP] = closestHit.normal;

            if (closestHit.collider != null)
            {
                surfaces |= CollidedSurface.Ceiling;

                if (closestHit.centroid.y - _collider2D.bounds.center.y < minDistanceFromEnv)
                {
                    transform.position += (minDistanceFromEnv -
                        (closestHit.centroid.y - _collider2D.bounds.center.y)) * Vector3.down;
                }
            }
        }

        if (forceCheck || vecToCheck.x >= -NEAR_ZERO)
        {
            // Right
            closestHit = GetClosestHit(_collider2D.bounds.center, Vector3.right, envCheckDistance);

            _collidersUpAgainst[DIRECTION_RIGHT] = closestHit.collider;
            _collidedNormals[DIRECTION_RIGHT] = closestHit.normal;

            if (closestHit.collider != null)
            {
                surfaces |= CollidedSurface.RightWall;

                if (closestHit.centroid.x - _collider2D.bounds.center.x < minDistanceFromEnv)
                {
                    transform.position += (minDistanceFromEnv -
                                          (closestHit.centroid.x - _collider2D.bounds.center.x)) * Vector3.left;
                }
            }
        }

        if (forceCheck ||
            -vecToCheck.y >= -NEAR_ZERO ||
            onSlope ||
            (HasFlag(CollidedSurface.Ground) && IsJumping()))
        {
            // Ground
            surfaces |= CheckGround(envCheckDistance, false);

            if (enableSlopes &&
                stickOnGround &&
                (IsOnGround() || IsSlipping()) &&
                surfaces == CollidedSurface.None)
            {
                surfaces |= CheckGround(distanceToCheckToStick, true);
            }
        }

        onSlope = false;

        if (enableSlopes)
        {
            // Slopes check
            if ((surfaces & (CollidedSurface.Ground | CollidedSurface.RightWall | CollidedSurface.LeftWall)) !=
                CollidedSurface.None)
            {
                // We only check for slopes if we are on the ground or colliding with left/right wall
                Vector2 dir = _bottomRight;
                Vector2 origin = new Vector2(_collider2D.bounds.max.x, _collider2D.bounds.min.y);
                Vector2 rightNormal = Vector2.zero;
                Vector2 leftNormal = Vector2.zero;

                closestHit = GetClosestHit(origin, dir, _cornerDistanceCheck, false);

                if (closestHit.collider != null && closestHit.normal.x < -NEAR_ZERO && Mathf.Abs(closestHit.normal.y) > NEAR_ZERO)
                {
                    surfaces |= CollidedSurface.SlopeRight;
                    rightNormal = closestHit.normal;
                }

                dir.x *= -1;
                origin = _collider2D.bounds.min;
                closestHit = GetClosestHit(origin, dir, _cornerDistanceCheck, false);

                if (closestHit.collider != null && closestHit.normal.x > NEAR_ZERO && Mathf.Abs(closestHit.normal.y) > NEAR_ZERO)
                {
                    surfaces |= CollidedSurface.SlopeLeft;
                    leftNormal = closestHit.normal;
                }

                if ((surfaces & CollidedSurface.SlopeLeft) != CollidedSurface.None &&
                    (surfaces & CollidedSurface.SlopeRight) != CollidedSurface.None)
                {
                    // Both sides are sloping if we can stand on a slope then we consider the least steep slope.
                    float leftDot = leftNormal.y;
                    float rightDot = rightNormal.y;

                    if (leftDot < _dotAllowedForSlopes && rightDot < _dotAllowedForSlopes)
                    {
                        // Would slip down both, consider just standing up.
                        surfaces &= ~(CollidedSurface.SlopeLeft | CollidedSurface.SlopeRight);
                        surfaces |= CollidedSurface.Ground;
                        _collidedNormals[DIRECTION_DOWN] = Vector2.up;
                    }
                    else if (leftDot >= _dotAllowedForSlopes && rightDot >= _dotAllowedForSlopes)
                    {
                        onSlope = true;

                        if (facingLeft)
                        {
                            slopeNormal = leftNormal;
                            surfaces &= ~CollidedSurface.SlopeRight;
                        }
                        else
                        {
                            slopeNormal = rightNormal;
                            surfaces &= ~CollidedSurface.SlopeLeft;
                        }
                    }
                    else if (leftDot >= _dotAllowedForSlopes)
                    {
                        onSlope = true;
                        slopeNormal = leftNormal;
                        surfaces &= ~CollidedSurface.SlopeRight;
                    }
                    else
                    {
                        onSlope = true;
                        slopeNormal = rightNormal;
                        surfaces &= ~CollidedSurface.SlopeLeft;
                    }
                }
                else if ((surfaces & CollidedSurface.SlopeRight) != CollidedSurface.None)
                {
                    slopeNormal = rightNormal;
                    onSlope = !((surfaces & CollidedSurface.Ground) != CollidedSurface.None &&
                               (_collidedNormals[DIRECTION_DOWN] == Vector2.up));
                }
                else if ((surfaces & CollidedSurface.SlopeLeft) != CollidedSurface.None)
                {
                    slopeNormal = leftNormal;
                    onSlope = !((surfaces & CollidedSurface.Ground) != CollidedSurface.None &&
                               (_collidedNormals[DIRECTION_DOWN] == Vector2.up));
                }
            }
        }

        return surfaces;
    }

    private float CalculateSpeedNeeded(float height)
    {
        return Mathf.Sqrt(-2 * height * gravityMultiplier * Physics2D.gravity.y);
    }

    private int GetFrameCount(float time)
    {
        return Mathf.RoundToInt(PC2D.Globals.GetFrameCount(time) / (timeScale != 0 ? timeScale : _savedTimeScale));
    }

    void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireCube(_ladderTopArea.center, _ladderTopArea.size);
        Gizmos.DrawWireCube(_ladderBottomArea.center, _ladderBottomArea.size);

        Gizmos.color = Color.red;
        Gizmos.DrawWireCube(_restrictedArea.center, _restrictedArea.size);

        Gizmos.color = Color.white;

        // Ground check.
        Bounds box = GetComponent<Collider2D>().bounds;
        Vector2 min;
        Vector2 max;

        // Ground check box
        min = box.min;
        max = box.max;
        min.y -= envCheckDistance;
        max.y = box.min.y;
        Gizmos.DrawWireCube(new Vector2((min.x + max.x) / 2, (min.y + max.y) / 2), new Vector2(max.x - min.x, min.y - max.y));

        // Left check box
        min = box.min;
        max = box.max;
        min.x -= envCheckDistance;
        max.x = box.min.x;
        Gizmos.DrawWireCube(new Vector2((min.x + max.x) / 2, (min.y + max.y) / 2), new Vector2(max.x - min.x, min.y - max.y));

        // Right check box
        min = box.min;
        max = box.max;
        min.x = box.max.x;
        max.x += envCheckDistance;
        Gizmos.DrawWireCube(new Vector2((min.x + max.x) / 2, (min.y + max.y) / 2), new Vector2(max.x - min.x, min.y - max.y));

        if (enableCornerGrabs)
        {
            min = box.min;
            max = box.max;
            min.y = max.y;
            max.y += cornerDistanceCheck;
            max.x = min.x;
            min.x -= cornerDistanceCheck;
            Gizmos.DrawWireCube(new Vector2((min.x + max.x) / 2, (min.y + max.y) / 2), new Vector2(max.x - min.x, min.y - max.y));

            min = box.min;
            max = box.max;
            min.y = max.y;
            max.y += cornerDistanceCheck;
            min.x = max.x;
            max.x += cornerDistanceCheck;
            Gizmos.DrawWireCube(new Vector2((min.x + max.x) / 2, (min.y + max.y) / 2), new Vector2(max.x - min.x, min.y - max.y));

            // Draw valid corner grab area
            Gizmos.color = Color.yellow;
            min = box.min;
            max = box.max;
            min.y = max.y - box.size.y * normalizedValidWallInteraction;
            min.x = max.x;
            max.x += cornerDistanceCheck;
            Gizmos.DrawWireCube(new Vector2((min.x + max.x) / 2, (min.y + max.y) / 2), new Vector2(max.x - min.x, min.y - max.y));

            min = box.min;
            max = box.max;
            min.y = max.y - box.size.y * normalizedValidWallInteraction;
            max.x = min.x;
            min.x -= cornerDistanceCheck;
            Gizmos.DrawWireCube(new Vector2((min.x + max.x) / 2, (min.y + max.y) / 2), new Vector2(max.x - min.x, min.y - max.y));
        }

        // Show the distance that it will take for the motor to stop on the ground and air.
        Vector2 from = new Vector2(box.max.x, box.min.y);
        Gizmos.color = Color.green;
        Gizmos.DrawLine(from, from + Vector2.right * groundStopDistance);

        from = box.max;
        Gizmos.color = Color.blue;
        Gizmos.DrawLine(from, from + Vector2.right * airStopDistance);

        if (movingPlatformDebug)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(_point, 0.01f);
            Gizmos.DrawWireCube(_prevPosPlat.center, _prevPosPlat.size);

            Gizmos.color = Color.blue;
            Gizmos.DrawWireCube(_startPosMotor.center, _startPosMotor.size);

            Gizmos.color = Color.green;
            Gizmos.DrawSphere(_point2, 0.01f);
            Gizmos.DrawWireCube(_movedPosMotor.center, _movedPosMotor.size);
        }

        if (iterationDebug && _iterationBounds != null)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawWireCube(_iterationBounds[0].center, _iterationBounds[0].size);

            Gizmos.color = Color.blue;

            for (int i = 2; i < _iterationsUsed + 2; i++)
            {
                Gizmos.DrawWireCube(_iterationBounds[i].center, _iterationBounds[i].size);
            }

            Gizmos.color = Color.green;
            Gizmos.DrawWireCube(_iterationBounds[1].center, _iterationBounds[1].size);
        }
    }

    private void ChangeState(MotorState newState)
    {
        // no change...
        if (motorState == newState)
        {
            return;
        }

        if (IsSlipping())
        {
            if (onSlippingEnd != null)
            {
                onSlippingEnd();
            }
        }
        if (IsJumping())
        {
            if (onJumpEnd != null)
            {
                onJumpEnd();
            }
        }
        if (IsDashing())
        {
            if (onDashEnd != null)
            {
                onDashEnd();
            }
        }
        if (IsDucking())
        {
            if(onDuckEnd != null)
            {
                onDuckEnd();
            }
        }

        // set
        motorState = newState;

        if (IsSlipping())
        {
            if (onSlipping != null)
            {
                onSlipping();
            }
        }
        if (IsJumping())
        {
            if (onJump != null)
            {
                onJump();
            }
        }
        if (IsDashing())
        {
            if (onDash != null)
            {
                onDash();
            }
        }
        if (IsDucking())
        {
            if(onDuck != null)
            {
                onDuck();
            }
        }
    }

}
