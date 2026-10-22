/**
 * @file xpcs.c
 * @brief XPCS (10 Gigabit Physical Coding Sublayer) Driver Implementation
 * 
 * This driver implements the XPCS functionality based on Synopsys DWC_xpcs IP.
 * It supports multiple speed modes from 1G to 25G, including:
 * - 1000BASE-X PCS
 * - 10GBASE-X PCS
 * - 10GBASE-R PCS
 * - 5GBASE-R PCS
 * - 2.5GBASE-X PCS
 * - USXGMII mode (10G/5G/2.5G)
 * 
 * Features:
 * - Auto-negotiation (Clause 37 and Clause 73)
 * - Energy Efficient Ethernet (EEE)
 * - Speed and duplex configuration
 * - Fault detection and debugging
 */


/* Platform-specific includes - to be replaced with actual platform headers */
#include "pcs.h"

/* Bit manipulation macro */
//#define BIT(x)                            (1U << (x))
#define MASK(high, low)                   (((1U << ((high) - (low) + 1)) - 1) << (low))

#define XPCS0_BASE_ADDR                   0x1A000
#define XPCS1_BASE_ADDR                   0x1B000

#define BITSTS_TIMEOUT_US                 10000
#define XPCS_EEE_TIMER                    0x100

/* PMA Registers */
#define SR_PMA_CTRL1                      0x10000
#define SR_PMA_KR_PMD_CTRL                0x10096
#define SR_PMA_KR_LP_CEU                  0x10098
#define SR_PMA_KR_LP_CESTS                0x10099
#define SR_PMA_KR_LD_CEU                  0x1009A
#define SR_PMA_KR_LD_CESTS                0x1009B

#define VR_PMA_KRTR_RX_EQ_CTRL            0x018009
#define VR_XS_PMA_MP_32G_TX_CM_CNTX_SEL0  0x01803C
#define VR_XS_PMA_MP_32G_TX_CNTX_CTRL0    0x01803E
#define VR_XS_PMA_MP_25G_TX_WIDTH_CTRL    0x018046
#define VR_XS_PMA_MP_12G_16G_25G_RX_GENCTRL0  0x018050
#define VR_XS_PMA_MP_32G_RX_CNTX_CTRL0    0x018092
#define VR_XS_PMA_MP_32G_LN_LINK_CTRL     0x018097
#define VR_XS_PMA_MP_12G_16G_25G_SRAM     0x01809B
#define VR_XS_PMA_SNPS_CR_CTRL            0x0180A0
#define VR_XS_PMA_SNPS_CR_ADDR            0x0180A1
#define VR_XS_PMA_SNPS_CR_DATA            0x0180A2
#define VR_XS_PMA_MP_25G_RX_WIDTH_CTRL    0x0180B0

/* XS PCS Registers */
#define SR_XS_PCS_CTRL1                   0x030000
#define SR_XS_PCS_STS1                    0x030001
#define SR_XS_PCS_CTRL2                   0x030007
#define SR_XS_PCS_EEE_ABL                 0x030014

#define VR_XS_PCS_DIG_CTRL1               0x038000
#define VR_XS_PCS_XAUI_CTRL               0x038004
#define VR_XS_PCS_DEBUG_CTRL              0x038005
#define VR_XS_PCS_EEE_MCTRL0              0x038006
#define VR_XS_PCS_KR_CTRL                 0x038007
#define VR_XS_PCS_EEE_TXTIMER             0x038008
#define VR_XS_PCS_EEE_RXTIMER             0x038009
#define VR_XS_PCS_DIG_STS                 0x038010

#define VR_XS_PCS_SFTY_UE_INTR0           0x0380F0
#define PSEQ_ERR                          BIT(15)
#define EF_UE_INT                         BIT(14)
#define EC_UE_INT                         BIT(13)
#define DPP_INT                           BIT(12)
#define PHY_IFT_INT                       BIT(11)
#define APB3_IFT_INT                      BIT(10)
#define CKOFT_INT                         BIT(9)
#define EEEFT_INT                         BIT(8)
#define CSRFT_INT                         BIT(7)
#define RXFT_INT                          BIT(6)
#define TXFT_INT                          BIT(5)
#define CKOFP_INT                         BIT(4)
#define EEEFP_INT                         BIT(3)
#define CSRFP_INT                         BIT(2)
#define RXFP_INT                          BIT(1)
#define TXFP_INT                          BIT(0)

/* AN Registers */
#define SR_AN_CTRL                        0x70000
#define VR_AN_INTR                        0x078002

/* MII Registers */
#define VR_MII_1_DIG_CTRL1                0x1A8000
#define VR_MII_2_DIG_CTRL1                0x1B8000
#define VR_MII_3_DIG_CTRL1                0x1C8000
#define SR_MII_CTRL                       0x1F0000
#define SR_MII_STS                        0x1F0001
#define SR_MII_AN_ADV                     0x1F0004

#define VR_MII_DIG_CTRL1                  0x1F8000
#define VR_MII_AN_CTRL                    0x1F8001
#define VR_MII_AN_INTR_STS                0x1F8002
#define VR_MII_EEE_MCTRL0                 0x1F8006
#define VR_MII_EEE_TXTIMER                0x1F8008
#define VR_MII_EEE_RXTIMER                0x1F8009
#define VR_MII_LINK_TIMER_CTRL            0x1F800A
#define VR_MII_MP_12G_16G_25G_SRAM        0x1F809B
#define VR_MII_SNPS_CR_CTRL               0x1F80A0
#define VR_MII_SNPS_CR_ADDR               0x1F80A1
#define VR_MII_SNPS_CR_DATA               0x1F80A2

/* Register bit field definitions */

/* SR_PMA_CTRL1 */
#define PMA_RST                         BIT(15)
#define PMA_SS13                        BIT(13)
#define PMA_LPM                         BIT(11)
#define PMA_SS6                         BIT(6)
#define PMA_SS_5_2                      MASK(5, 2)
#define PMA_LB                          BIT(0)

/* SR_PMA_KR_PMD_CTRL */
#define TR_EN                           BIT(1)
#define RS_TR                           BIT(0)

/* VR_XS_PMA_MP_12G_16G_25G_SRAM */
#define EXT_LD_DN                       BIT(1)

/* VR_XS_PMA_SNPS_CR_CTRL */
#define START_BUSY                      BIT(0)
#define WR_RDN                          BIT(1)

/* SR_XS_PCS_CTRL1 */
#define CTL_RESET                       BIT(15)
#define LBE                             BIT(14)
#define SS13                            BIT(13)
#define AN_ENABLE                       BIT(12)
#define LPM                             BIT(11)
#define CS_EN                           BIT(10)
#define XAUI_STOP                       BIT(9)
#define RESTART_AN                      BIT(9)
#define DUPLEX_MODE                     BIT(8)
#define SS6                             BIT(6)
#define SS5                             BIT(5)
#define SS_5_2                          MASK(5, 2)

/* SR_XS_PCS_STS1 */
#define RLU                             BIT(2)

/* SR_XS_PCS_CTRL2 */
#define XPCS_TYPE_SEL                   MASK(3, 0)
#define XPCS_TYPE_KX4_KX		0x1
#define XPCS_TYPE_10GBASE_R             0x0
#define XPCS_TYPE_5GBASE_R              0xF
#define XPCS_TYPE_2DOT5GBASE_X          0xE

/* SR_XS_PCS_EEE_ABL */
#define KREEE                           BIT(6)
#define KX4EEE                          BIT(5)
#define KXEEE                           BIT(4)

/* VR_XS_PCS_DIG_CTRL1 */
#define USXG_EN                         BIT(9)
#define CL37_BP                         BIT(12)
#define USRA_RST                        BIT(10)
#define EN_2DOT5G_MODE                  BIT(2)

/* VR_XS_PCS_XAUI_CTRL */
#define XAUI_MODE                       BIT(0)

/* VR_XS_PCS_DEBUG_CTRL */
#define SUPRESS_LOS_DET                 BIT(4)
#define RX_DT_ENCTL                     BIT(6)

/* VR_XS_PCS_EEE_MCTRL0 */
#define LTX_EN                          BIT(0)
#define LRX_EN                          BIT(1)
#define EEE_SLR_BYP                     BIT(5)

/* VR_XS_PCS_KR_CTRL */
#define USXG_MODE                       MASK(12, 10)
#define USXG_MODE_10G_SXGMII            0x0
#define USXG_MODE_5G_SXGMII             0x1
#define USXG_MODE_2DOT5G_SXGMII         0x2
#define USXG_MODE_10G_DXGMII            0x3
#define USXG_MODE_5G_DXGMII             0x4
#define USXG_MODE_10G_QXGMII            0x5

/* VR_XS_PCS_DIG_STS */
#define PSEQ_STATE                      MASK(4, 2)
#define PSEQ_POWER_GOOD                 0x4
#define PSEQ_POWER_DOWN                 0x6
#define PSEQ_POWER_SAVE                 0x5

/* SR_AN_CTRL */
#define AN_EN                           BIT(12)

/* VR_MII_DIG_CTRL1 */
#define CL37_TMR_OVR_RIDE               BIT(3)

/* VR_MII_AN_CTRL */
#define MII_AN_INTR_EN                  BIT(0)
#define MII_CTRL                        BIT(8)
#define TX_CONFIG                       BIT(3)
#define SGMII_LINK_STS                  BIT(4)
#define PCS_MODE_MASK                   MASK(2, 1)
#define PCS_MODE_1GBASEX                0x0
#define PCS_MODE_SGMII                  0x2
#define PCS_MODE_QSGMII                 0x3

/* VR_MII_AN_INTR_STS */
#define CL37_ANCMPLT_INTR               BIT(0)
#define USXG_AN_STS_LINK                BIT(14)
#define USXG_AN_STS_DUPLEX              BIT(13)
#define USXG_AN_STS_SPEED               MASK(12, 10)
#define USXG_SPEED_10G                  0x3
#define USXG_SPEED_5G                   0x5
#define USXG_SPEED_2DOT5G               0x4
#define USXG_AN_STS_EEE                 BIT(9)
#define USXG_AN_STS_EEE_CLK             BIT(8)

/* SR_MII_CTRL */
#define MII_RST                         BIT(15)
#define MII_LBE                         BIT(14)
#define MII_SS13                        BIT(13)
#define MII_AN_ENABLE                   BIT(12)
#define MII_LPM                         BIT(11)
#define MII_RESTART_AN                  BIT(9)
#define MII_DUPLEX_MODE                 BIT(8)
#define MII_SS6                         BIT(6)
#define MII_SS5                         BIT(5)

/* SR_MII_STS */
#define LINK_STS                        BIT(2)

/* VR_MII_SNPS_CR_CTRL */
#define MII_START_BUSY                  BIT(0)
#define MII_WR_RDN                      BIT(1)

struct pcs_device xpcs0 = {
    .base_addr = (void *)XPCS0_BASE_ADDR,
    .current_speed = PCS_SPEED_10G,
    .current_duplex = PCS_FULL_DUPLEX,
    .link_up = 0,
    .an_enabled = 0,
    .eee_enabled = 0,
    .kr_training_enabled = 0,
    .max_speed = PCS_SPEED_10G,
    .pcs_mode = PCS_TYPE_10GBASE_R,
};

struct pcs_device xpcs1 = {
    .base_addr = (void *)XPCS1_BASE_ADDR,
    .current_speed = PCS_SPEED_10G,
    .current_duplex = PCS_FULL_DUPLEX,
    .link_up = 0,
    .an_enabled = 0,
    .eee_enabled = 0,
    .kr_training_enabled = 0,
    .max_speed = PCS_SPEED_10G,
    .pcs_mode = PCS_TYPE_10GBASE_R,
};

/**
 * @brief Write to Synopsys PHY register via CR port
 * 
 * @param base_addr Base address of XPCS
 * @param phy_addr PHY register address
 * @param data Data to write
 * @param is_xs true for XS_PMA, false for MII
 * @return 0 on success, -1 on failure
 */
s32 xpcs_cr_port_write(struct pcs_device *dev, u32 phy_addr, 
                         u32 data, u32 is_xs)
{
    u32 ctrl_reg = is_xs ? VR_XS_PMA_SNPS_CR_CTRL : VR_MII_SNPS_CR_CTRL;
    u32 addr_reg = is_xs ? VR_XS_PMA_SNPS_CR_ADDR : VR_MII_SNPS_CR_ADDR;
    u32 data_reg = is_xs ? VR_XS_PMA_SNPS_CR_DATA : VR_MII_SNPS_CR_DATA;
    
    /* Step 1: Wait for bit 0 to be 0 */
    if (wait_reg_clear(dev, ctrl_reg, START_BUSY, 1000) != 0) {
        dev_err(dev->dev, "CR port busy timeout");
        return -1;
    }
    
    /* Step 2: Write PHY address */
    pcs_write_reg(dev, addr_reg, phy_addr);
    
    /* Step 3: Write data */
    pcs_write_reg(dev, data_reg, data);
    
    /* Step 4: Set bit 1 (WR_RDN) */
    pcs_write_reg(dev, ctrl_reg, WR_RDN);
    
    /* Step 5: Set bit 0 (START_BUSY) */
    pcs_write_reg(dev, ctrl_reg, WR_RDN | START_BUSY);
    
    /* Step 6: Wait for bit 0 to clear */
    if (wait_reg_clear(dev, ctrl_reg, START_BUSY, 1000) != 0) {
        dev_err(dev->dev, "CR port write timeout");
        return -1;
    }
    
    return 0;
}

/**
 * @brief Read from Synopsys PHY register via CR port
 * 
 * @param base_addr Base address of XPCS
 * @param phy_addr PHY register address
 * @param data Pointer to store read data
 * @param is_xs true for XS_PMA, false for MII
 * @return 0 on success, -1 on failure
 */
s32 xpcs_cr_port_read(struct pcs_device *dev, u32 phy_addr, 
                        u32 *data, u32 is_xs)
{
    u32 ctrl_reg = is_xs ? VR_XS_PMA_SNPS_CR_CTRL : VR_MII_SNPS_CR_CTRL;
    u32 addr_reg = is_xs ? VR_XS_PMA_SNPS_CR_ADDR : VR_MII_SNPS_CR_ADDR;
    u32 data_reg = is_xs ? VR_XS_PMA_SNPS_CR_DATA : VR_MII_SNPS_CR_DATA;
    
    /* Step 1: Wait for bit 0 to be 0 */
    if (wait_reg_clear(dev, ctrl_reg, START_BUSY, 1000) != 0) {
        dev_err(dev->dev, "CR port busy timeout");
        return -1;
    }
    
    /* Step 2: Write PHY address */
    pcs_write_reg(dev, addr_reg, phy_addr);
    
    /* Step 3: Clear bit 1 (WR_RDN = 0 for read) */
    pcs_write_reg(dev, ctrl_reg, 0);
    
    /* Step 4: Set bit 0 (START_BUSY) */
    pcs_write_reg(dev, ctrl_reg, START_BUSY);
    
    /* Step 5: Wait for bit 0 to clear */
    if (wait_reg_clear(dev, ctrl_reg, START_BUSY, 1000) != 0) {
        dev_err(dev->dev, "CR port read timeout");
        return -1;
    }
    
    /* Step 6: Read data */
    *data = pcs_read_reg(dev, data_reg);
    
    return 0;
}

/**
 * @brief Configure speed bits in control register
 * 
 * Speed encoding:
 * - SS5=1, SS6=0, SS13=1 -> 5 Gbps
 * - SS5=1, SS6=0, SS13=0 -> 2.5 Gbps
 * - SS5=0, SS6=1, SS13=1 -> 10 Gbps
 * - SS5=0, SS6=1, SS13=0 -> 1000 Mbps
 * - SS5=0, SS6=0, SS13=1 -> 100 Mbps
 * - SS5=0, SS6=0, SS13=0 -> 10 Mbps
 * 
 * @param speed Speed enumeration
 * @param ss5 Pointer to SS5 bit value
 * @param ss6 Pointer to SS6 bit value
 * @param ss13 Pointer to SS13 bit value
 */
static void get_speed_bits(pcs_speed_t speed, u32 *ss5, 
                           u32 *ss6, u32 *ss13)
{
    switch (speed) {
    case PCS_SPEED_10M:
        *ss5 = 0; *ss6 = 0; *ss13 = 0;
        break;
    case PCS_SPEED_100M:
        *ss5 = 0; *ss6 = 0; *ss13 = 1;
        break;
    case PCS_SPEED_1G:
        *ss5 = 0; *ss6 = 1; *ss13 = 0;
        break;
    case PCS_SPEED_2DOT5G:
        *ss5 = 1; *ss6 = 0; *ss13 = 0;
        break;
    case PCS_SPEED_5G:
        *ss5 = 1; *ss6 = 0; *ss13 = 1;
        break;
    case PCS_SPEED_10G:
        *ss5 = 0; *ss6 = 1; *ss13 = 1;
        break;
    case PCS_SPEED_25G:
        *ss5 = 0; *ss6 = 0; *ss13 = 0; /* Default */
        break;
    default:
	break;
    }
}

static void xpcs_32g_phy_config(struct pcs_device *dev)
{
    u32 ctrl_reg;
    u32 cntx_val;

    switch(dev->pcs_mode)
    {
        case PCS_TYPE_5GBASE_R:
            cntx_val = 0x2;
            break;

        case PCS_TYPE_2DOT5GBASE_X:
            cntx_val = 0x1;
            break;

        case PCS_TYPE_1GBASE_X:
            cntx_val = 0x0;
            break;

        case PCS_TYPE_1G_SGMII:
            cntx_val = 0x6;
            break;
        
        case PCS_TYPE_10GBASE_R:
        default:
            cntx_val = 0x3;
            break;
    }

    /* Configure RX/TX context_sel_ID */
    ctrl_reg = pcs_read_reg(dev, VR_XS_PMA_MP_32G_RX_CNTX_CTRL0);
    ctrl_reg |= cntx_val; 
    pcs_write_reg(dev, VR_XS_PMA_MP_32G_RX_CNTX_CTRL0, ctrl_reg);

    ctrl_reg = pcs_read_reg(dev, VR_XS_PMA_MP_32G_TX_CNTX_CTRL0);
    ctrl_reg |= cntx_val; 
    pcs_write_reg(dev, VR_XS_PMA_MP_32G_TX_CNTX_CTRL0, ctrl_reg);

    ctrl_reg = pcs_read_reg(dev, VR_XS_PMA_MP_32G_TX_CM_CNTX_SEL0);
    ctrl_reg |= cntx_val; 
    pcs_write_reg(dev, VR_XS_PMA_MP_32G_TX_CM_CNTX_SEL0, ctrl_reg);

    /* Configure RX/TX width to 10-bit */
    ctrl_reg = pcs_read_reg(dev, VR_XS_PMA_MP_25G_RX_WIDTH_CTRL);
    ctrl_reg &= ~MASK(2, 0);
    ctrl_reg |= 0x1; /* 3'b001 for 10-bit */
    pcs_write_reg(dev, VR_XS_PMA_MP_25G_RX_WIDTH_CTRL, ctrl_reg);
    
    ctrl_reg = pcs_read_reg(dev, VR_XS_PMA_MP_25G_TX_WIDTH_CTRL);
    ctrl_reg &= ~MASK(2, 0);
    ctrl_reg |= 0x1; /* 3'b001 for 10-bit */
    pcs_write_reg(dev, VR_XS_PMA_MP_25G_TX_WIDTH_CTRL, ctrl_reg);
}


/**
 * @brief Switch XPCS to 1G or KX mode
 * 
 * Follows the sequence from section 3.6:
 * 1. Set 1000BASE-X mode in SR_XS_PCS_CTRL2
 * 2. Configure RXAUI if needed
 * 3. Disable 2.5G mode
 * 4. Configure SGMII if needed
 * 5. Configure PHY for 1.25 Gbps
 * 6-10: Power state transition
 * 
 * @param xpcs Pointer to XPCS device structure
 * @param pcs_mode PCS mode (1GBASE-X, SGMII, etc.)
 * @return 0 on success, negative error code on failure
 */
static s32 xpcs_set_1g_mode(struct pcs_device *dev)
{
    u32 ctrl_reg;
    
    /* Step 1: Set 1000BASE-X mode */
    if(dev->pcs_mode == PCS_TYPE_1GBASE_X){
        ctrl_reg = pcs_read_reg(dev, SR_XS_PCS_CTRL2);
        ctrl_reg &= ~XPCS_TYPE_SEL;
        ctrl_reg |= XPCS_TYPE_KX4_KX; /* 0001 for 1000BASE-X */
        pcs_write_reg(dev, SR_XS_PCS_CTRL2, ctrl_reg);
    }
    else if(dev->pcs_mode == PCS_TYPE_1G_SGMII){
        /* Step 4: Configure SGMII mode if needed */
        ctrl_reg = pcs_read_reg(dev, VR_MII_AN_CTRL);
        ctrl_reg &= ~PCS_MODE_MASK;
        ctrl_reg |= SS6;
        ctrl_reg &= (~SS13);
        pcs_write_reg(dev, VR_MII_AN_CTRL, ctrl_reg);
    }
    else{
        dev_err(dev->dev, "Current PCS mode invalid!");
        return -1;
    }
    
    /* Step 3: Disable 2.5G mode */
    ctrl_reg = pcs_read_reg(dev, VR_XS_PCS_DIG_CTRL1);
    ctrl_reg &= ~EN_2DOT5G_MODE;
    pcs_write_reg(dev, VR_XS_PCS_DIG_CTRL1, ctrl_reg);
    
    /* Step 5: Configure PHY for 1.25 Gbps */
    xpcs_32g_phy_config(dev);
    
    //TODO: LINK_NUM_0 how to config?

    /* Step 6-10: Power state transition */
    /* Step 6: Set LPM */
    ctrl_reg = pcs_read_reg(dev, SR_XS_PCS_CTRL1);
    ctrl_reg |= LPM;
    pcs_write_reg(dev, SR_XS_PCS_CTRL1, ctrl_reg);
    
    /* Step 7: Wait for power down */
    if (wait_reg_set(dev, VR_XS_PCS_DIG_STS, PSEQ_STATE, 10000) != 0) {
        dev_err(dev->dev, "Power down timeout");
        return -1;
    }
    
    /* Step 8: Clear LPM */
    ctrl_reg = pcs_read_reg(dev, SR_XS_PCS_CTRL1);
    ctrl_reg &= ~LPM;
    pcs_write_reg(dev, SR_XS_PCS_CTRL1, ctrl_reg);
    
    /* Step 9: Wait for power good */
    if (wait_reg_clear(dev, VR_XS_PCS_DIG_STS, PSEQ_STATE, 10000) != 0) {
        dev_err(dev->dev, "Power good timeout");
        return -1;
    }
    
    /* Step 10: Clear debug control bits */
    ctrl_reg = pcs_read_reg(dev, VR_XS_PCS_DEBUG_CTRL);
    ctrl_reg &= ~(SUPRESS_LOS_DET | RX_DT_ENCTL);
    pcs_write_reg(dev, VR_XS_PCS_DEBUG_CTRL, ctrl_reg);
    
    dev->current_speed = PCS_SPEED_1G;
    
    return 0;
}

static void xpcs_get_usxgmii_mode(struct pcs_device *dev, u32 * usxg_mode)
{
    switch(dev->pcs_mode)
    {
        case PCS_TYPE_5GBASE_R:
            *usxg_mode = USXG_MODE_5G_SXGMII;
            break;

        case PCS_TYPE_2DOT5GBASE_X:
            *usxg_mode = USXG_MODE_2DOT5G_SXGMII;
            break;

        case PCS_TYPE_10GBASE_R:
            *usxg_mode = USXG_MODE_10G_SXGMII;
            break;

        default:
            dev_err(dev->dev, "Unexpected pcs_mode!");
            break;
    }
}

/**
 * @brief Set link speed
 * 
 * @param xpcs Pointer to XPCS device structure
 * @param speed Speed enumeration
 * @return 0 on success, negative error code on failure
 */
static s32 xpcs_set_speed(struct pcs_device *dev, pcs_speed_t speed)
{
    u32 ctrl_reg;
    u32 ss5 = 0;
    u32 ss6 = 0;
    u32 ss13 = 0;
    
    get_speed_bits(speed, &ss5, &ss6, &ss13);

    ctrl_reg = pcs_read_reg(dev, SR_MII_CTRL);
    ctrl_reg &= ~(SS5 | SS6 | SS13);
    ctrl_reg |= (ss5 << 5) | (ss6 << 6) | (ss13 << 13);
    pcs_write_reg(dev, SR_MII_CTRL, ctrl_reg);

    dev->current_speed = speed;
    
    return 0;
}


/**
 * @brief Switch XPCS to USXGMII mode (10G/5G/2.5G)
 * 
 * Follows the sequence from section 3.5:
 * 1. Set BASE-R mode in SR_XS_PCS_CTRL2
 * 2. Enable USXGMII mode
 * 3. Select USXGMII sub-mode
 * 4. Configure PHY for target speed
 * 5. Power state transition
 * 6. Configure backbone Ethernet
 * 7. Configure MII AN control
 * 8-16. Complete auto-negotiation and setup
 * 
 * @param xpcs Pointer to XPCS device structure
 * @param speed Target speed to set (XPCS_USXG_10G, XPCS_USXG_5G, XPCS_USXG_2DOT5G)
 * @return 0 on success, negative error code on failure
 */
static s32 xpcs_set_usxgmii_mode(struct pcs_device *dev, pcs_speed_t speed)
{
    u32 ctrl_reg;
    u32 usxg_mode;
    
    /* Step 1: Set BASE-R mode */
    ctrl_reg = pcs_read_reg(dev, SR_XS_PCS_CTRL2);
    ctrl_reg &= ~XPCS_TYPE_SEL;
    ctrl_reg |= XPCS_TYPE_10GBASE_R; /* 0000 for BASE-R */
    pcs_write_reg(dev, SR_XS_PCS_CTRL2, ctrl_reg);
    
    /* Step 2: Enable USXGMII mode */
    ctrl_reg = pcs_read_reg(dev, VR_XS_PCS_DIG_CTRL1);
    ctrl_reg |= USXG_EN;
    pcs_write_reg(dev, VR_XS_PCS_DIG_CTRL1, ctrl_reg);
    
    /* Step 3: Select USXGMII sub-mode */
    ctrl_reg = pcs_read_reg(dev, VR_XS_PCS_KR_CTRL);
    ctrl_reg &= ~USXG_MODE;
    xpcs_get_usxgmii_mode(dev, &usxg_mode);
    ctrl_reg |= (usxg_mode << 10);
    pcs_write_reg(dev, VR_XS_PCS_KR_CTRL, ctrl_reg);
    
    /* Step 4: Configure PHY for target speed */
    xpcs_32g_phy_config(dev);
    
    /* Step 5: Power state transition */
    /* Step 5.5: Set LPM */
    ctrl_reg = pcs_read_reg(dev, SR_XS_PCS_CTRL1);
    ctrl_reg |= LPM;
    pcs_write_reg(dev, SR_XS_PCS_CTRL1, ctrl_reg);
    
    /* Step 5.6: Wait for power down state */
    if (wait_reg_set(dev, VR_XS_PCS_DIG_STS, PSEQ_STATE, 10000) != 0) {
        dev_err(dev->dev, "Power down timeout");
        return -1;
    }
    
    /* Step 5.7: Clear LPM */
    ctrl_reg = pcs_read_reg(dev, SR_XS_PCS_CTRL1);
    ctrl_reg &= ~LPM;
    pcs_write_reg(dev, SR_XS_PCS_CTRL1, ctrl_reg);
    
    /* Step 5.8: Wait for power good state */
    if (wait_reg_clear(dev, VR_XS_PCS_DIG_STS, PSEQ_STATE, 10000) != 0) {
        dev_err(dev->dev, "Power good timeout");
        return -1;
    }
    
    /* Step 5.9: Set debug control bits */
    ctrl_reg = pcs_read_reg(dev, VR_XS_PCS_DEBUG_CTRL);
    ctrl_reg |= SUPRESS_LOS_DET | RX_DT_ENCTL;
    pcs_write_reg(dev, VR_XS_PCS_DEBUG_CTRL, ctrl_reg);
    
    /* Step 6: Configure backplane Ethernet */
    ctrl_reg = pcs_read_reg(dev, SR_AN_CTRL);
    ctrl_reg &= ~AN_EN;
    pcs_write_reg(dev, SR_AN_CTRL, ctrl_reg);
    
    ctrl_reg = pcs_read_reg(dev, VR_XS_PCS_DIG_CTRL1);
    ctrl_reg |= CL37_BP;
    pcs_write_reg(dev, VR_XS_PCS_DIG_CTRL1, ctrl_reg);
    
    /* Step 7: Configure MII AN control */
    ctrl_reg = 0;
    ctrl_reg |= MII_AN_INTR_EN; /* Enable interrupt (optional) */
    ctrl_reg |= MII_CTRL; /* Set MII_CTRL based on capability */
    ctrl_reg |= TX_CONFIG; /* TX_CONFIG: 1 for PHY side, 0 for MAC side */
    ctrl_reg |= SGMII_LINK_STS; /* Indicate link status for PHY side */
    pcs_write_reg(dev, VR_MII_AN_CTRL, ctrl_reg);
    
    /* Step 9: Configure speed for PHY side */
    xpcs_set_speed(dev, speed);
    
    /* Step 10: Enable Clause 37 auto-negotiation */
    ctrl_reg = pcs_read_reg(dev, SR_MII_CTRL);
    ctrl_reg |= MII_AN_ENABLE;
    pcs_write_reg(dev, SR_MII_CTRL, ctrl_reg);
    
    /* Step 11-13: Wait for AN completion and clear interrupt */
    /* This will be handled by interrupt or polling */
    
    /* Step 15: Wait for XGMII clock stability */
    dev->osi_core->osd_ops.udelay(1);
    
    /* Step 16: Assert and wait for USRA_RST */
    ctrl_reg = pcs_read_reg(dev, VR_XS_PCS_DIG_CTRL1);
    ctrl_reg |= USRA_RST;
    pcs_write_reg(dev, VR_XS_PCS_DIG_CTRL1, ctrl_reg);
    
    /* Wait for auto-clear */
    if (wait_reg_clear(dev, VR_XS_PCS_DIG_CTRL1, USRA_RST, 1000) != 0) {
        dev_err(dev->dev, "USRA_RST timeout");
        return -1;
    }
    
    dev->current_speed = speed;
    dev->link_up = 1;
    
    return 0;
}



/**
 * @brief Initialize XPCS device
 * 
 * Follows the initialization sequence from section 3.3:
 * 1. Power on with reset asserted
 * 2. Wait for PHY requirements
 * 3. Release reset
 * 4. Load firmware and context to SRAM via CR
 * 5. Wait for reset completion
 * 6. Configure debug settings for KR mode
 * 7. Configure backplane Ethernet if needed
 * 8. Configure Clause 37 auto-negotiation if needed
 * 9. Wait for link status
 * 
 * @param xpcs Pointer to XPCS device structure
 * @return 0 on success, negative error code on failure
 */
int xpcs_init(struct osi_core_priv_data *osi_core)
{
    u32 ctrl_reg;
    s32 ret = 0;
    struct pcs_device *dev = NULL;

    if(osi_core->instance_id == 0){
	    dev = &xpcs0;
    }
    else if(osi_core->instance_id == 1){
	    dev = &xpcs1;
    }
    else{
	    dev_err(osi_core->dev, "%s can not support xpcs\n", osi_core->if_name);
	    return -1;
    }

    osi_core->pcs_dev = dev;
    dev->dev = osi_core->dev;
    dev->base_addr = osi_core->xpcs_base;       /* Base address of XPCS registers */
    dev->current_speed = osi_core->speed;       /* Current link speed */
    dev->current_duplex = PCS_FULL_DUPLEX;      /* Current duplex mode */
    dev->link_up = 1;                      	/* Link status */
    dev->an_enabled = 1;                   	/* Auto-negotiation enabled */
    dev->eee_enabled = 1;                  	/* EEE enabled */
    dev->fec_enabled = 1;
    dev->kr_training_enabled = 1;          	/* KR training enabled (for Clause 72) */
    dev->flags = 0;                    		/* Device flags */
    dev->max_speed = PCS_SPEED_10G;            	/* Maximum supported speed */
    dev->pcs_mode = osi_core->phy_iface_mode;	/* PCS mode */
    dev->osi_core = osi_core;

    //TODO: will be deleted
    return 0;

    if (!dev)
        return -1;
    
    /* Step 1-3: Power on and release reset (handled by hardware) */
    /* Assuming reset is already released when this function is called */
    
    /* Step 4: Load firmware and context to SRAM */
    /* Wait at least 10 clock cycles for cr_para_clk */
    dev->osi_core->osd_ops.udelay(1);
    
    /* Perform CR write operations to load SRAM */
    /* This is placeholder - actual firmware loading depends on context_sel_ID.txt */
    
    /* Set EXT_LD_DN bit after SRAM load, For 1000BASE-X only configuration */
    if(dev->pcs_mode == PCS_TYPE_1GBASE_X)
    {
        ctrl_reg = pcs_read_reg(dev, VR_MII_MP_12G_16G_25G_SRAM);
        ctrl_reg |= EXT_LD_DN;
        pcs_write_reg(dev, VR_MII_MP_12G_16G_25G_SRAM, ctrl_reg);
        /* Wait for reset bit to clear */
        if (wait_reg_clear(dev, SR_MII_CTRL, MII_RST, 10000) != 0) {
            dev_err(dev->dev, "MII reset timeout");
            ret = -1;
            goto err_out;
        }
    }
    else{ //else pcs mode
        ctrl_reg = pcs_read_reg(dev, VR_XS_PMA_MP_12G_16G_25G_SRAM);
        ctrl_reg |= EXT_LD_DN;
        pcs_write_reg(dev, VR_XS_PMA_MP_12G_16G_25G_SRAM, ctrl_reg);
        if (wait_reg_clear(dev, SR_XS_PCS_CTRL1, CTL_RESET, 10000) != 0) {
            dev_err(dev->dev, "XPCS reset timeout");
            ret = -1;
            goto err_out;
        }
    }

    /* Ignore Step 6: Configure debug settings for 10BASE-(K)R mode */

    /* Step 7: Configure backplane Ethernet if needed */
    /* This depends on configuration - disable AN if not needed */
    ctrl_reg = pcs_read_reg(dev, SR_AN_CTRL);
    ctrl_reg &= ~AN_EN; /* Disable Clause 73 AN by default */
    pcs_write_reg_safe(dev, SR_AN_CTRL, ctrl_reg);
    ctrl_reg = pcs_read_reg(dev, VR_XS_PCS_DIG_CTRL1);
    ctrl_reg |= CL37_BP;
    ret = pcs_write_reg_safe(dev, VR_XS_PCS_DIG_CTRL1, ctrl_reg);
    if (ret != 0) {
        goto err_out;
    }

    if(dev->pcs_mode == PCS_TYPE_1GBASE_X ||
        dev->pcs_mode == PCS_TYPE_1G_SGMII)
    {
        xpcs_set_1g_mode(dev);
    }
    else{
        xpcs_set_usxgmii_mode(dev, dev->current_speed);
    }
    /* Step 9: Wait for link status */
    /* For 1000BASE-X, wait LINK_STS bit */
    if(dev->pcs_mode == PCS_TYPE_1GBASE_X){
        if (wait_reg_set(dev, SR_MII_STS, LINK_STS, BITSTS_TIMEOUT_US) != 0) {
            dev_err(dev->dev, "Link up timeout");
            ret = -1;
            goto err_out;
        }
    }
    else{/* For non-1000BASE-X, wait RLU bit */
        if(wait_reg_set(dev, SR_XS_PCS_STS1, RLU, BITSTS_TIMEOUT_US) != 0){
            dev_err(dev->dev, "RX Link up timeout");
            ret = -1;
            goto err_out;
        }
    }
    
    dev->an_enabled = 1;
    dev->link_up = 1;
    
err_out:
    return ret;
}

/**
 * @brief Enable Clause 37 auto-negotiation
 * 
 * @param dev Pointer to XPCS device structure
 * @param enable true to enable, false to disable
 * @return 0 on success, negative error code on failure
 */
static s32 xpcs_cl37_an_enable(struct pcs_device *dev, u32 enable)
{
    u32 ctrl_reg, an_ctrl_reg;
    
    ctrl_reg = pcs_read_reg(dev, SR_MII_CTRL);
    
    if (enable) {
        ctrl_reg |= MII_AN_ENABLE;
        /* Trigger auto-negotiation */
        pcs_write_reg(dev, SR_MII_CTRL, ctrl_reg);
        dev->an_enabled = 1;
    } else {
        ctrl_reg &= ~MII_AN_ENABLE;
        pcs_write_reg(dev, SR_MII_CTRL, ctrl_reg);
        dev->an_enabled = 0;
    }

    /* set auto-nego complete intr */
    an_ctrl_reg = pcs_read_reg(dev, VR_MII_AN_CTRL);
    an_ctrl_reg |= MII_AN_INTR_EN;
    pcs_write_reg(dev, VR_MII_AN_CTRL, an_ctrl_reg);

    return 0;
}

/**
 * @brief Check auto-negotiation status
 * 
 * @param dev Pointer to XPCS device structure
 * @param status Pointer to store AN status
 * @return 0 on success, negative error code on failure
 */
static s32 xpcs_an_get_status(struct pcs_device *dev, u32 *status)
{
    *status = pcs_read_reg(dev, VR_MII_AN_INTR_STS);
    
    return 0;
}

/**
 * @brief Clear auto-negotiation interrupt
 * 
 * @param dev Pointer to XPCS device structure
 * @return 0 on success, negative error code on failure
 */
s32 xpcs_an_clear_intr(struct pcs_device *dev)
{
    /* Write 0 to bit 0 to clear interrupt */
    pcs_write_reg(dev, VR_MII_AN_INTR_STS, CL37_ANCMPLT_INTR);
    
    return 0;
}

/**
 * @brief Get link status
 * 
 * @param xpcs Pointer to XPCS device structure
 * @return true if link is up, false otherwise
 */
u32 xpcs_get_link_status(struct pcs_device *dev)
{
    u32 sts_reg;
    
    /* Check appropriate status register based on configuration */
    if (dev->current_speed == PCS_SPEED_1G) {
        sts_reg = pcs_read_reg(dev, SR_MII_STS);
        if (sts_reg & RLU) {
            dev->link_up = 1;
            return 1;
        }
    }
    else {
        sts_reg = pcs_read_reg(dev, SR_XS_PCS_STS1);
        if (sts_reg & LINK_STS) {
            dev->link_up = 1;
            return 1;
        }
    }

    dev->link_up = 0;
    return 0;
}

/**
 * @brief Set duplex mode
 * 
 * @param xpcs Pointer to XPCS device structure
 * @param duplex Duplex mode
 * @return 0 on success, negative error code on failure
 */
s32 xpcs_set_duplex(struct pcs_device *dev, pcs_duplex_t duplex)
{
    u32 ctrl_reg;

    ctrl_reg = pcs_read_reg(dev, SR_MII_CTRL);
    
    if (duplex == PCS_FULL_DUPLEX) {
        ctrl_reg |= MII_DUPLEX_MODE;
    } else {
        ctrl_reg &= ~MII_DUPLEX_MODE;
    }
    
    pcs_write_reg(dev, SR_MII_CTRL, ctrl_reg);
    
    dev->current_duplex = duplex;
    
    return 0;
}

/**
 * @brief xpcs_start - Start XPCS
 *
 * Algorithm: This routine enables AN and set speed based on AN status
 *
 * @param[in] xpcs: Pointer to XPCS device structure
 *
 * @retval 0 on success
 * @retval -1 on failure.
 */
s32 xpcs_start(struct pcs_device *dev)
{
    u32 an_status;
    pcs_speed_t speed;
    pcs_duplex_t duplex_mode;
    u32 link_status = 0;
    s32 ret;

    /* Enable Clause 37 auto-negotiation if needed */
    xpcs_cl37_an_enable(dev, 1);

    //wait for an complete
    xpcs_an_get_status(dev, &an_status);
    speed = ((an_status & USXG_AN_STS_SPEED) >> 10);
    duplex_mode = ((an_status & USXG_AN_STS_DUPLEX) >> 13);
    link_status = ((an_status & USXG_AN_STS_LINK) >> 14);

    ret = xpcs_set_speed(dev, speed);
    return ret;    
}

/**
 * @brief Check EEE capability
 * 
 * @param dev Pointer to XPCS device structure
 * @return EEE capability bitmap
 */
static u32 xpcs_check_eee_capability(struct pcs_device *dev)
{   
    return pcs_read_reg(dev, SR_XS_PCS_EEE_ABL);
}

/**
 * @brief Enable/Disable EEE
 * 
 * Follows the sequence from section 3.10:
 * 1. Check EEE capability
 * 2. Configure timers based on clk_eee_i frequency
 * 3. Enable FEC fast sync if needed
 * 4. Enable TX EEE
 * 5. Enable RX EEE
 * 
 * @param dev Pointer to XPCS device structure
 * @param tx_enable Enable TX EEE
 * @param rx_enable Enable RX EEE
 * @param fec_enable Enable FEC fast sync
 * @return 0 on success, negative error code on failure
 */
int xpcs_eee(struct pcs_device *dev, u32 en_dis)
{
    u32 eee_abl;
    u32 ctrl_reg;
   
   return 0;

    /* Step 1: Check EEE capability */
    eee_abl = xpcs_check_eee_capability(dev);
    
    if (!eee_abl) {
        dev_err(dev->osi_core->dev, "EEE not supported");
        if(en_dis)
        {
            return -1;
        }
        else{
            return 0;
        }
    }
    
    if(en_dis)
    {
        /* Step 2: Configure timers */
        /* Timer values depend on clk_eee_i frequency */
        /* This is configuration-specific - placeholder */
        ctrl_reg = XPCS_EEE_TIMER;
        pcs_write_reg(dev, VR_XS_PCS_EEE_TXTIMER, ctrl_reg);
        pcs_write_reg(dev, VR_XS_PCS_EEE_RXTIMER, ctrl_reg);
                
        ctrl_reg = pcs_read_reg(dev, VR_XS_PCS_EEE_MCTRL0);

        /* Step 3: Enable FEC fast sync if needed */
        ctrl_reg |= EEE_SLR_BYP;

        /* Step 4: Enable TX EEE */
        ctrl_reg |= LTX_EN;

        /* Step 5: Enable RX EEE */
        ctrl_reg |= LRX_EN;
    }
    else{
        ctrl_reg = pcs_read_reg(dev, VR_XS_PCS_EEE_MCTRL0);
        //Disable FEC fast sync if needed
        ctrl_reg &= ~EEE_SLR_BYP;

        //Disable TX EEE
        ctrl_reg &= ~LTX_EN;

        //Disable RX EEE
        ctrl_reg &= ~LRX_EN;        
    }
    pcs_write_reg(dev, VR_XS_PCS_EEE_MCTRL0, ctrl_reg);
    
    dev->eee_enabled = en_dis;
    
    return 0;
}


/**
 * @brief Reset XPCS device
 * 
 * @param dev Pointer to XPCS device structure
 * @return 0 on success, negative error code on failure
 */
int xpcs_reset(struct pcs_device *dev)
{
    u32 ctrl_reg;
    
    /* Assert reset */
    ctrl_reg = pcs_read_reg(dev, SR_XS_PCS_CTRL1);
    ctrl_reg |= CTL_RESET;
    pcs_write_reg(dev, SR_XS_PCS_CTRL1, ctrl_reg);
    
    dev->osi_core->osd_ops.udelay(10);
    
    /* Release reset */
    ctrl_reg &= ~CTL_RESET;
    pcs_write_reg(dev, SR_XS_PCS_CTRL1, ctrl_reg);
    
    /* Wait for reset to complete */
    if (wait_reg_clear(dev, SR_XS_PCS_CTRL1, CTL_RESET, 10000) != 0) {
        return -1;
    }
    
    return 0;
}

/**
 * @brief Get current speed
 * 
 * @param dev Pointer to XPCS device structure
 * @return Current speed enumeration
 */
pcs_speed_t xpcs_get_speed(struct pcs_device *dev)
{
    return dev->current_speed;
}

/**
 * @brief Get current duplex mode
 * 
 * @param dev Pointer to XPCS device structure
 * @return Current duplex mode
 */
pcs_duplex_t xpcs_get_duplex(struct pcs_device *dev)
{
    return dev->current_duplex;
}

void xpcs_ue_intr_handle(struct pcs_device * xpcs)
{
    xpcs->last_err = pcs_read_reg(xpcs, VR_XS_PCS_SFTY_UE_INTR0);
    pcs_write_reg(xpcs, VR_XS_PCS_SFTY_UE_INTR0, 0x0);
}

void xpcs_ce_intr_handle(struct pcs_device *xpcs)
{
    dev_err(xpcs->dev, "%s handle xpcs sfty ce interrupt\n", xpcs->osi_core->if_name);
}

void xpcs_sbd_intr_handle(struct pcs_device *xpcs)
{
    dev_err(xpcs->dev, "%s handle xpcs sbd interrupt\n", xpcs->osi_core->if_name);
}
