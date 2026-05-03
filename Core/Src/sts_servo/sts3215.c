/*
 * sts3215.c
 *
 *  Created on: Apr 11, 2026
 *      Author: miyab
 */


#include "sts_servo/sts3215.h"
#include "sts_servo/sts_packet.h"

static sts_status_t sts_convert_status(sts_bus_status_t s)
{
    switch (s) {
    case STS_BUS_OK:
        return STS_OK;
    case STS_BUS_TIMEOUT:
        return STS_TIMEOUT;
    case STS_BUS_BAD_RESPONSE:
    case STS_BUS_BAD_CHECKSUM:
        return STS_PROTOCOL_ERROR;
    default:
        return STS_ERROR;
    }
}

void sts3215_init(sts3215_t *servo, sts_bus_t *bus, uint8_t id)
{
    if (servo == NULL) {
        return;
    }

    servo->bus = bus;
    servo->id = id;
}

sts_status_t sts3215_ping(sts3215_t *servo)
{
    if (servo == NULL || servo->bus == NULL) {
        return STS_ERROR;
    }

    uint8_t tx[STS_MAX_PACKET_SIZE];
    uint8_t tx_len = 0;
    uint8_t rx[STS_MAX_PACKET_SIZE];
    uint16_t rx_len = 0;

    if (!sts_build_packet(servo->id, STS_INST_PING, NULL, 0, tx, &tx_len)) {
        return STS_ERROR;
    }

    sts_bus_status_t s = sts_bus_send(servo->bus, tx, tx_len);
    if (s != STS_BUS_OK) {
        return sts_convert_status(s);
    }

    s = sts_bus_receive_status(servo->bus, servo->id, rx, sizeof(rx), &rx_len);
    return sts_convert_status(s);
}

sts_status_t sts3215_read_u8(sts3215_t *servo, uint8_t address, uint8_t *value)
{
    if (servo == NULL || servo->bus == NULL || value == NULL) {
        return STS_ERROR;
    }

    uint8_t params[2] = { address, 1 };
    uint8_t tx[STS_MAX_PACKET_SIZE];
    uint8_t tx_len = 0;
    uint8_t rx[STS_MAX_PACKET_SIZE];
    uint16_t rx_len = 0;

    if (!sts_build_packet(servo->id, STS_INST_READ, params, 2, tx, &tx_len)) {
        return STS_ERROR;
    }

    sts_bus_status_t s = sts_bus_send(servo->bus, tx, tx_len);
    if (s != STS_BUS_OK) {
        return sts_convert_status(s);
    }

    s = sts_bus_receive_status(servo->bus, servo->id, rx, sizeof(rx), &rx_len);
    if (s != STS_BUS_OK) {
        return sts_convert_status(s);
    }

    // status packet: FF FF ID LEN ERR PARAM... CHK
    // 1 byte read -> total len should be 7
    if (rx_len < 7) {
        return STS_PROTOCOL_ERROR;
    }

    if (rx[4] != 0) {
        return STS_PROTOCOL_ERROR;
    }

    *value = rx[5];
    return STS_OK;
}

sts_status_t sts3215_read_u16(sts3215_t *servo, uint8_t address, uint16_t *value)
{
    if (servo == NULL || servo->bus == NULL || value == NULL) {
        return STS_ERROR;
    }

    uint8_t params[2] = { address, 2 };
    uint8_t tx[STS_MAX_PACKET_SIZE];
    uint8_t tx_len = 0;
    uint8_t rx[STS_MAX_PACKET_SIZE];
    uint16_t rx_len = 0;

    if (!sts_build_packet(servo->id, STS_INST_READ, params, 2, tx, &tx_len)) {
        return STS_ERROR;
    }

    sts_bus_status_t s = sts_bus_send(servo->bus, tx, tx_len);
    if (s != STS_BUS_OK) {
        return sts_convert_status(s);
    }

    s = sts_bus_receive_status(servo->bus, servo->id, rx, sizeof(rx), &rx_len);
    if (s != STS_BUS_OK) {
        return sts_convert_status(s);
    }

    if (rx_len < 8) {
        return STS_PROTOCOL_ERROR;
    }

    if (rx[4] != 0) {
        return STS_PROTOCOL_ERROR;
    }

    // STS/SMS系は low byte first で扱う資料・実装が一般的
    *value = (uint16_t)rx[5] | ((uint16_t)rx[6] << 8);
    return STS_OK;
}

sts_status_t sts3215_write_u8(sts3215_t *servo, uint8_t address, uint8_t value){
    if (servo == NULL || servo->bus == NULL) {
        return STS_ERROR;
    }

    uint8_t params[2];
    params[0] = address;
    params[1] = value;

    uint8_t tx[STS_MAX_PACKET_SIZE];
    uint8_t tx_len = 0;

    uint8_t rx[STS_MAX_PACKET_SIZE];
    uint16_t rx_len = 0;

    if (!sts_build_packet(
    		servo->id,
			STS_INST_WRITE,
			params,
			2,
			tx,
			&tx_len))
    {
        return STS_ERROR;
    }

    sts_bus_status_t s;

    s = sts_bus_send(servo->bus, tx, tx_len);
    if (s != STS_BUS_OK) {
        return sts_convert_status(s);
    }

    s = sts_bus_receive_status(
    		servo->bus,
			servo->id,
			rx,
			sizeof(rx),
			&rx_len);

    return sts_convert_status(s);
}

sts_status_t sts3215_write_u16(sts3215_t *servo, uint8_t address, uint16_t value){
    if (servo == NULL || servo->bus == NULL) {
        return STS_ERROR;
    }

    uint8_t params[3];

    params[0] = address;

    params[1] = (uint8_t)(value & 0xFF);       // low
    params[2] = (uint8_t)((value >> 8) & 0xFF); // high

    uint8_t tx[STS_MAX_PACKET_SIZE];
    uint8_t tx_len = 0;

    uint8_t rx[STS_MAX_PACKET_SIZE];
    uint16_t rx_len = 0;

    if (!sts_build_packet(
            servo->id,
            STS_INST_WRITE,
            params,
            3,
            tx,
            &tx_len))
    {
        return STS_ERROR;
    }

    sts_bus_status_t s;

    s = sts_bus_send(servo->bus, tx, tx_len);
    if (s != STS_BUS_OK) {
        return sts_convert_status(s);
    }

    s = sts_bus_receive_status(
            servo->bus,
            servo->id,
            rx,
            sizeof(rx),
            &rx_len);

    return sts_convert_status(s);
}

sts_status_t sts3215_set_torque_enable(sts3215_t *servo, bool enable){
    uint8_t value;

    if (enable)
        value = 1;
    else
        value = 0;

    return sts3215_write_u8(
            servo,
            STS_ADDR_TORQUE_ENABLE,
            value);
}

sts_status_t sts3215_set_goal_position(sts3215_t *servo, uint16_t position){
    return sts3215_write_u16(
            servo,
            STS_ADDR_GOAL_POSITION,
            position);
}

sts_status_t sts3215_stop(sts3215_t *servo)
{
    uint16_t present_position;

    if (servo == NULL || servo->bus == NULL) {
        return STS_ERROR;
    }

    if (sts3215_read_u16(
            servo,
            STS_ADDR_PRESENT_POSITION,
            &present_position) != STS_OK) {
        return STS_ERROR;
    }

    return sts3215_set_goal_position(
            servo,
            present_position);
}

sts_status_t sts3215_is_reached(sts3215_t *servo,
                                uint16_t target_position,
                                uint16_t tolerance,
                                bool *reached)
{
    uint16_t present_position;
    int32_t error;

    if (servo == NULL || servo->bus == NULL || reached == NULL) {
        return STS_ERROR;
    }

    sts_status_t status = sts3215_read_u16(
            servo,
            STS_ADDR_PRESENT_POSITION,
            &present_position);

    if (status != STS_OK) {
        return status;
    }

    error = (int32_t)target_position - (int32_t)present_position;

    if (error < 0) {
        error = -error;
    }

    if ((uint16_t)error <= tolerance) {
        *reached = true;
    } else {
        *reached = false;
    }

    return STS_OK;
}
