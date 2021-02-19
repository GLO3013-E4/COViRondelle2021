<template>
  <v-card class="grey lighten-3">
    <v-card-title sm="6" class="grey darken-1 d-flex justify-center">
      <h5 class="white--text">Grip state</h5>
    </v-card-title>
    <div ref="gripState" class="d-flex justify-center font-weight-bold">
      <v-switch
        ref="switch"
        v-model="puckInGrip"
        inset
        :label="puckInGrip ? 'puck in grip' : 'no puck'"
      ></v-switch>
    </div>
  </v-card>
</template>
<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import { mapState } from 'vuex';

@Component({
  computed: {
    ...mapState(['puckInGrip']),
  },
})
export default class PuckInClamp extends Vue {
  public puckInGrip!: boolean;
}
</script>
