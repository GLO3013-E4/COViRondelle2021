<template>
  <div>
    <v-card class="grey lighten-3">
      <v-card-title class="grey darken-1 d-flex justify-center">
        <h5 class="white--text">{{ $t(`consumptions.robotConsumption`) }}</h5>
      </v-card-title>
      <v-container>
        <v-row align="center">
          <v-col sm="6">
            <h5>{{ $t(`consumptions.wheel1`) }} : {{ wheel1Info }}</h5>
            <h5>{{ $t(`consumptions.wheel2`) }} : {{wheel2Info}}</h5>
          </v-col>
          <v-col sm="6">
            <h5>{{ $t(`consumptions.wheel3`) }} : {{wheel3Info}} </h5>
            <h5>{{ $t(`consumptions.wheel4`) }} : {{wheel4Info}} </h5>
            </v-col>
          <v-col sm="12" class="servoMotorAndTotal">
            <h5>{{ $t(`consumptions.servoMotor`) }} : {{servoMotorInfo}}</h5>
            <h5>{{ $t(`consumptions.total`) }} : {{totalInfo}}</h5>
          </v-col>
        </v-row>
      </v-container>
    </v-card>
  </div>
</template>

<script lang="ts">
import { RobotConsumption } from '@/types/robotConsumption';
import { Component, Vue } from 'vue-property-decorator';
import { mapState } from 'vuex';

@Component({
  computed: {
    ...mapState(['robotConsumption', 'batteryConsumption']),
  },
})
export default class BatteryCharge extends Vue {
  public robotConsumption!: RobotConsumption;

  public get wheel1Info(){
      return `${this.robotConsumption.wheel1} Watts`;
  }
  public get wheel2Info(){
      return `${this.robotConsumption.wheel2} Watts`
  }
public get wheel3Info(){
      return `${this.robotConsumption.wheel3} Watts`
  }
  public get wheel4Info(){
      return `${this.robotConsumption.wheel4} Watts`
  }
    public get servoMotorInfo(){
      return `${this.robotConsumption.servoMotor} Watts`
  }
      public get totalInfo(){
      return `${this.robotConsumption.total} Watts`
  }
}
</script>

<style scoped>
.v-card__title {
  padding: 0em;
}

.servoMotorAndTotal{
  padding-top: 0.5em;
}
</style>
